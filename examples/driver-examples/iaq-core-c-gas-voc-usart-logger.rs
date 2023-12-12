//!  Substantially modified for rtic 0.6
//!
//! Measures the CO2 and TVOC equivalents in the air with an iAQ-Core-C module,
//! logs the values and sends them through the serial interface every 10 seconds.
//!
//! To setup the serial communication, have a look at the discovery book:
//! https://rust-embedded.github.io/discovery/10-serial-communication/index.html
//!
//! This is the hardware configuration for the STM32F103 "Bluepill" board using I2C1 and USART1.
//!
//! ```
//! BP   <-> iAQ-Core-C <-> Serial module
//! GND  <-> GND        <-> GND
//! 3.3V <-> VCC        <-> VDD
//! PB8  <-> SCL     
//! PB9  <-> SDA     
//! PB6                 <-> RX
//! ```
//!
//! Run with:
//! `cargo embed --example iaq-core-c-gas-voc-usart-logger`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

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
    use iaq_core::{IaqCore, Measurement};

    //use cortex_m::asm; //asm::delay(N:u32) blocks the program for at least N CPU cycles.
                       //delay_ms could be used but needs to use a timer other than Systick
                       //use embedded_hal::blocking::delay; //delay::delay_ms(N:u32) blocks the program for N ms.

    use core::fmt::Write;
    
    use systick_monotonic::*;

    use nb::block;
    use rtt_target::{rprintln, rtt_init_print};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    const MONOTICK: u32 = 100;
    const PERIOD: u64 = 10;  // used as seconds
    //const PERIOD: Duration<T, NOM, DENOM> = 10.secs();
    
    use rust_integration_testing_of_examples::dht_i2c_led_usart_delay::{
        setup_dht_i2c_led_usart_delay_using_dp, TxType, I2cType, LED, LedType, MONOCLOCK};


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[shared]
    struct Shared {
        led: LedType,
        //sensor:  IaqCore<I2cProxy<'static,   Mutex<RefCell<I2cType>>>, Ccs811Mode::App>,
        sensor:  IaqCore<I2cProxy<'static,   Mutex<RefCell<I2cType>>>>,
        //sensor: IaqCore<I2cType>,
        tx: TxType,
    }

    #[local]
    struct Local {
        led_state: bool,
        index: usize,
        measurements: [Measurement; 2400],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
        let mono = Systick::new(cx.core.SYST, MONOCLOCK);

        rtt_init_print!();
        rprintln!("iAQ-Core-C example");

        //let device: Peripherals = cx.device;

        let (_dht, i2c, mut led, mut tx, _delay) = setup_dht_i2c_led_usart_delay_using_dp(cx.device);

        led.off();

        let led_state: bool = false;
        let index: usize = 0;
        let measurements: [Measurement; 2400] = [Measurement {
            co2: 0,
            tvoc: 0,
            resistance: 0,
        }; 2400];

        // rtic needs task sharing not provided by BusManagerSimple: 
        let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

        let sensor = IaqCore::new(manager.acquire_i2c());

        measure::spawn_after(PERIOD.secs()).unwrap();

        writeln!(tx, "start\r",).unwrap();

        (Shared {led, sensor, tx}, Local {led_state, index, measurements}, init::Monotonics(mono),
        )
    }

    #[task(shared = [led, sensor, tx], local = [led_state, index, measurements,])]
    fn measure(mut cx: measure::Context) {
        if *cx.local.led_state {
            cx.shared.led.lock(|led| led.off());
            *cx.local.led_state = false;
        } else {
            cx.shared.led.lock(|led| led.on());
            *cx.local.led_state = true;
        }

        let default = Measurement {
            co2: 1,
            tvoc: 1,
            resistance: 1,
        };
        if *cx.local.index < cx.local.measurements.len() {
            let data = cx
                .shared
                .sensor
                .lock(|sensor| block!(sensor.data()))
                .unwrap_or(default);
            cx.local.measurements[*cx.local.index] = data;
            *cx.local.index += 1;
        }
        for i in 0..*cx.local.index {
            let data = cx.local.measurements[i];
            //writeln!(cx.resources.tx,"{},{},{},{}\r",i, data.co2, data.tvoc, data.resistance).unwrap();
            cx.shared
                .tx
                .lock(|tx| writeln!(tx, "{},{},{},{}\r", i, data.co2, data.tvoc, data.resistance))
                .unwrap();
        }
        measure::spawn_after(PERIOD.secs()).unwrap();
    }
}
