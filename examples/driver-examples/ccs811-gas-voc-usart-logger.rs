//!
//! Continuously measure the eCO2 and eTVOC in the air, logs the values and sends
//! them through the serial interface every 10 seconds.
//! In order to compensate for the ambient temperature and humidity, an HDC2080
//! sensor is used.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
//!
//! To setup the serial communication, have a look at the discovery book:
//! https://rust-embedded.github.io/discovery/10-serial-communication/index.html
//!
//! This is the hardware configuration for the STM32F103 "Bluepill" board using I2C1 and USART1.
//! Much of the setup is done in modules led and i2c in scr/i2c.rs and scr/led.rs
//!
//! ```
//! BP   <-> CCS811 <-> HDC2080 <-> Serial module
//! GND  <-> GND    <-> GND     <-> GND
//! 3.3V <-> VCC    <-> VCC     <-> VDD
//! PB8  <-> SCL    <-> SCL      
//! PB9  <-> SDA    <-> SDA      
//! PB6             <-> RX
//! GND  <-> nWAKE
//! 3.3V <-> RST
//! ```
//!
//! Run with:
//! `cargo embed --example ccs811-gas-voc-usart-logger`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_rtt_target as _;

use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use embedded_ccs811::{
        mode as Ccs811Mode, prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode,
        SlaveAddr as Ccs811SlaveAddr,
    };

    use hdc20xx::{mode as Hdc20xxMode, Hdc20xx, SlaveAddr as Hdc20xxSlaveAddr};

    //uuse cortex_m_semihosting::{hprintln};
    use rtt_target::{rprintln, rtt_init_print};

    use core::fmt::Write; 
    use nb::block;

    use rtic;
    use rtic_monotonics::systick::Systick;
    use rtic_monotonics::systick::fugit::{ExtU32};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    const PERIOD: u32 = 10;  // used as seconds
    
    //use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};
    use rust_integration_testing_of_examples::dht_i2c_led_usart_delay::{
         setup_dht_i2c_led_usart_delay_using_dp, LED, LedType, 
         I2cType, TxType, MONOCLOCK};

    // cortex_m::asm  delay is only used in init. 
    // See other examples for the case when a shared delay is needed.
    use cortex_m::asm; //asm::delay(N:u32) blocks the program for at least N CPU cycles.
                       //delay_ms could be used but needs to use a timer other than Systick
                       //use embedded_hal::blocking::delay; //delay::delay_ms(N:u32) blocks the program for N ms.



    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {

        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        rtt_init_print!();
        rprintln!("CCS811/HDC2080 example");

        //let device: Peripherals = cx.device;
        //let (i2c, mut led, mut tx) = setup(device);
        let (_dht, i2c, mut led, mut tx, _delay) = setup_dht_i2c_led_usart_delay_using_dp(cx.device);

        led.off();

        // Previously these were initialized static mut in fn measure()
        let led_state: bool;
        let env: [(f32, f32); 1200] = [(0.0, 0.0); 1200];
        let index: usize = 0;
        let measurements: [AlgorithmResult; 1200] = [AlgorithmResult {
            eco2: 0,
            etvoc: 0,
            raw_current: 0,
            raw_voltage: 0,
        }; 1200];

        // rtic needs task sharing not provided by BusManagerSimple: 
        let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

        let mut hdc2080 = Hdc20xx::new(manager.acquire_i2c(), Hdc20xxSlaveAddr::default());
        let mut ccs811 = Ccs811Awake::new(manager.acquire_i2c(), Ccs811SlaveAddr::default());
        ccs811.software_reset().unwrap();

        // Delay while ccs811 resets.
        // Note that this delay cannot use SYST because Monotonics uses that (for spawn,
        //   although spawn has not yet happened so there may be a way?)

        led.on();
        asm::delay(3 * MONOCLOCK); // (3 * MONOCLOCK cycles give aprox 3 second delay
                               //delay::delay_ms(3_000_u16);
        led.off();
        led_state = false;

        let mut ccs811 = ccs811.start_application().ok().unwrap();
        let en = block!(hdc2080.read()).unwrap();
        ccs811
            .set_environment(en.temperature, en.humidity.unwrap_or(0.0))
            .unwrap();
        ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

        measure::spawn().unwrap();

        writeln!(tx, "start\r",).unwrap();

        (Shared {led, ccs811, hdc2080, tx}, Local {led_state, env, index, measurements})
    }

    #[shared]
    struct Shared {
        led: LedType,
        ccs811:  Ccs811Awake<I2cProxy<'static,   Mutex<RefCell<I2cType>>>, Ccs811Mode::App>,
        hdc2080:     Hdc20xx<I2cProxy<'static,   Mutex<RefCell<I2cType>>>, Hdc20xxMode::OneShot>,
        tx: TxType,
    }

    #[local]
    struct Local {
        led_state: bool,
        env: [(f32, f32); 1200],
        index: usize,
        measurements: [AlgorithmResult; 1200],
    }


    #[task(shared = [led, ccs811, hdc2080, tx], local = [led_state, env, index, measurements])]
    async fn measure(mut cx: measure::Context) {
       loop {
           Systick::delay(PERIOD.secs()).await;

           if *cx.local.led_state {
               cx.shared.led.lock(|led| led.off());
               *cx.local.led_state = false;
           } else {
               cx.shared.led.lock(|led| led.on());
               *cx.local.led_state = true;
           }

           let default = AlgorithmResult::default();
           if *cx.local.index < cx.local.measurements.len() {
               //let data = block!(cx.shared.ccs811.data()).unwrap_or(default);
               let data = cx
                   .shared
                   .ccs811
                   .lock(|ccs811| block!(ccs811.data()))
                   .unwrap_or(default);
               cx.local.measurements[*cx.local.index] = data;
               //let en = block!(cx.shared.hdc2080.read()).unwrap();
               let en = cx
                   .shared
                   .hdc2080
                   .lock(|hdc2080| block!(hdc2080.read()))
                   .unwrap();
               let temp = en.temperature;
               let humidity = en.humidity.unwrap_or(0.0);
               cx.local.env[*cx.local.index] = (temp, humidity);
               *cx.local.index += 1;
               //cx.shared.ccs811.set_environment(temp, humidity).unwrap();
               cx.shared
                   .ccs811
                   .lock(|ccs811| ccs811.set_environment(temp, humidity))
                   .unwrap();
           }
           //writeln!(cx.shared.tx, "\rstart\r",).unwrap();
           cx.shared.tx.lock(|tx| writeln!(tx, "\rstart\r",)).unwrap();
           for i in 0..*cx.local.index {
               let data = cx.local.measurements[i];
               let en = if i == 0 {
                   (0.0, 0.0)
               } else {
                   cx.local.env[i - 1]
               };
               //writeln!(cx.shared.tx,  "{},{},{},{},{},{:.2},{:.2}\r", i, data.eco2,
               //        data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1 ).unwrap();
               cx.shared
                   .tx
                   .lock(|tx| {
                       writeln!(
                           tx,
                           "{},{},{},{},{},{:.2},{:.2}\r",
                           i, data.eco2, data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1
                       )
                   })
                   .unwrap();
           }
       }
    }
}
