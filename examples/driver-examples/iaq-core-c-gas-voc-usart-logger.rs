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
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use rtic;
    use rtic_monotonics::systick::Systick;
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};
    use rtic_monotonics::systick::fugit::{ExtU32};

    use core::fmt::Write; 
    use nb::block;

    /////////////////////   iaq
    use iaq_core::{IaqCore, Measurement};

    /////////////////////   hals


    /////////////////////  bus sharing
  
    //use shared_bus::{I2cProxy};
    //use core::cell::RefCell;
    //use cortex_m::interrupt::Mutex;


    /////////////////////   boards
    use rust_integration_testing_of_examples::monoclock::MONOCLOCK;
    use rust_integration_testing_of_examples::led::{LED, LedType};
    use rust_integration_testing_of_examples::i2c::{I2c1Type as I2cType};
    use rust_integration_testing_of_examples::opendrain_i2c_led_usart::{TxType};
    use rust_integration_testing_of_examples::opendrain_i2c_led_usart;


    /////////////////////  

    const PERIOD: u32 = 10;  // used as seconds
    
    #[shared]
    struct Shared {
        led: LedType,
        sensor: IaqCore<I2cType>,
        tx: TxType,
    }

    #[local]
    struct Local {
        led_state: bool,
        index: usize,
        measurements: [Measurement; 2400],
    }

    /////////////////////  

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        //rtt_init_print!();
        //rprintln!("example");
        hprintln!("iaq-core-c-gas-voc... example").unwrap();

        let (_pin, i2cset, mut led, mut tx, _delay, _clocks) = opendrain_i2c_led_usart::setup_from_dp(cx.device);

        led.off();
        let led_state: bool = false;

        let index: usize = 0;
        let measurements: [Measurement; 2400] = [Measurement {
            co2: 0,
            tvoc: 0,
            resistance: 0,
        }; 2400];

        let sensor = IaqCore::new(i2cset);

        measure::spawn().unwrap();

        writeln!(tx, "start\r",).unwrap();
//      for byte in b"\r\nstart\r\n" {
//          #[cfg(feature = "stm32f4xx")]
//          block!(tx.write(*byte)).ok();
//          #[cfg(not(feature = "stm32f4xx"))]
//          block!(tx.write_byte(*byte)).ok();
//      }

        (Shared {led, sensor, tx}, Local {led_state, index, measurements})
    }

    #[task(shared = [led, sensor, tx], local = [led_state, index, measurements])]
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
               //writeln!(cx.shared.tx,"{},{},{},{}\r",i, data.co2, data.tvoc, data.resistance).unwrap();
               cx.shared
                   .tx
                   .lock(|tx| writeln!(tx, "{},{},{},{}\r", i, data.co2, data.tvoc, data.resistance))
                   .unwrap();
           }
       }
    }
}
