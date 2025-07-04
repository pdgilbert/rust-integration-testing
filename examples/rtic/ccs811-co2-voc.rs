//!  Building and run tested on Blackpill, Feb 17, 2022
//!
//!  CLEANUP DESCRIPTION. DISPLAY OR LOG???
//! This example is derived from driver-examples/ccs811-gas-voc-usart-logger.rs
//! which comes from examples by Diego Barrios Romero.
//! See the introductory blog post with some pictures here:
//! https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
//!
//! Below has been substantially modified for rtic 1.0.0 and to run with various HALs,
//! and to use a DHT11 sensor to compensate for the ambient temperature and humidity,
//! and to disable logging.
//! (See driver-examples/ccs811-gas-voc-usart-logger.rs for hdc2080
//!   temperature and humidity sensor which uses i2c. That example also keeps logging info.)

//! Continuously measure the eCO2 and eTVOC in the air, 
//! DISABLED logs the values and sends
//! DISABLED them through the serial interface every 10 seconds.
//!
//! The hardware configuration for the STM32F103 "Bluepill" board uses 
//!   PB6 for RX,  PB8 for SCL, PB9  <-> SDA,  nWAKE to GND, RST to 3.3v.
//! See the setup code for other boards.

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
use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000); 

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

    use embedded_ccs811::{
        mode as Ccs811Mode, prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode,
        SlaveAddr as Ccs811SlaveAddr,
    };
 
    //https://github.com/michaelbeaumont/dht-sensor
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::{blocking::read, Reading};
    #[cfg(feature = "dht22")]
    use dht_sensor::dht22::{blocking::read, Reading};
    //use dht_sensor::*;

    use core::fmt::Write;

    use nb::block;

    // set up for shared bus even though only one i2c device is used here
    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    const READ_INTERVAL: u32 = 10;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::
                         setup::{MONOCLOCK, OpenDrainType, I2cType, LedType, LED, TxType, Delay, prelude::*,};



    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
     
        Mono::start(cx.core.SYST, MONOCLOCK);

        //rtt_init_print!();
        //rprintln!("CCS811 example");

        let (mut dht, i2c, mut led, mut tx, mut delay) = setup::pin_i2c_led_tx_delay_from_dp(cx.device);

        led.on(); 
        delay.delay_ms(1000);
        led.off();
        
        // intial dht reading
        let (temperature, humidity) = match read(&mut delay, &mut dht) {  //NEEDS NON SYSTICK DELAY
            Ok(Reading {temperature, relative_humidity,})
               =>  {hprintln!("temperature:{}, humidity:{}, ", temperature, relative_humidity).unwrap();
                    (temperature, relative_humidity)
                   },
            Err(e) 
               =>  {hprintln!("dht Error {:?}. Using default temperature:{}, humidity:{}", e, 25, 40).unwrap(); 
                    //panic!("Error reading DHT"),
                    (25, 40)  //supply default values
                   },
        };
        delay.delay_ms(2000); //  2 second delay for dhtsensor initialization

        // initialize ccs811
        //let env: [(f32, f32); 1200] = [(0.0, 0.0); 1200];
        //let index: usize = 0;
        //let measurements: [AlgorithmResult; 1200] = [AlgorithmResult {
        //    eco2: 0, etvoc: 0, raw_current: 0, raw_voltage: 0, }; 1200];

        //  SHARED BUS COMPILES HERE on stm32f4xx  (WITH Ccs811Awake NOT I2CDisplayInterface) Jan 2024
        //  shared-bus = {  git = "https://github.com/Rahix/shared-bus", features = ["cortex-m"] } // compiles Feb 10, 2024
        //  shared-bus = { version = "0.2.2", features = ["cortex-m"] }                            // compiles Feb 10, 2024
        //  shared-bus = { version = "0.3.1", features = ["cortex-m"] }                            // compiles Feb 10, 2024

        let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

        //    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
        //    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        //        .into_buffered_graphics_mode();

        let mut ccs811 = Ccs811Awake::new(manager.acquire_i2c(), Ccs811SlaveAddr::default());
        hprintln!("let mut ccs811 = Ccs811Awake::new(").unwrap();
        ccs811.software_reset().unwrap();
        hprintln!("_reset").unwrap();

        delay.delay_ms(3000);  // Delay while ccs811 resets
        hprintln!("delay.delay_ms(3000u32)").unwrap();

        let mut ccs811 = ccs811.start_application().ok().unwrap();
        hprintln!("let mut ccs811 = ccs811.start_application(").unwrap();
        ccs811.set_environment(temperature.into(), humidity.into()).unwrap(); //i8 into f32, u8 into f32
        ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();
        hprintln!("ccs811.set_mode(3000u32)").unwrap();

        // make certain this does not start sooner than end of systick timer led check above
        measure::spawn().unwrap();

        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();
        writeln!(tx, "start\r",).unwrap();

        (Shared {led}, Local {dht, ccs811, tx, delay })
    }

    #[shared]
    struct Shared {
        led: LedType,
        //manager??? ,  uses i2c:   I2c1Type, or text_style, display ??
        //env: [(f32, f32); 1200],
        //index: usize,
        //measurements: [AlgorithmResult; 1200],
    }


    #[local]
    struct Local {
        dht: OpenDrainType,
        ccs811: Ccs811Awake<I2cProxy<'static,   Mutex<RefCell<I2cType>>>, Ccs811Mode::App>,
        tx: TxType,
        delay:Delay,
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). It may affect debugging.
           rtic::export::wfi()
        }
    }

    #[task(shared = [led,], local = [dht, ccs811, tx, delay ], priority=1 )]
    async fn measure(cx: measure::Context) {

       // this might be nicer if read could be done by spawn rather than wait for delay
       let delay = cx.local.delay;
       let dht = cx.local.dht;

       loop {
           Mono::delay(READ_INTERVAL.secs()).await;

           //hprintln!("measure").unwrap();
           blink::spawn(BLINK_DURATION).ok();

           let z = read(delay, dht);
           let (temperature, humidity) = match z {
               Ok(Reading {temperature, relative_humidity,})
                  =>  {hprintln!("temperature:{}, humidity:{}, ", temperature, relative_humidity).unwrap();
                       (temperature, relative_humidity)
                      },
               Err(e) 
                  =>  {hprintln!("dht Error {:?}. Using default temperature:{}, humidity:{}", e, 25, 40).unwrap(); 
                       //panic!("Error reading DHT"),
                       (25, 40)  //supply default values
                      },
           };
           //hprintln!("temperature:{}, humidity:{}, ", temperature, humidity).unwrap();

           //let data = cx.share.ccs811.lock(|ccs811| block!(ccs811.data())).unwrap_or(AlgorithmResult::default());
           let data = block!(cx.local.ccs811.data()).unwrap_or(AlgorithmResult::default());
           hprintln!("ccs811 data eco2:{}, etvoc:{}, raw_current:{}, raw_volt:{}", 
                             data.eco2, data.etvoc, data.raw_current, data.raw_voltage).unwrap();

           //cx.share.ccs811.lock(|ccs811| ccs811.set_environment(temperature.into(), humidity.into())).unwrap();
           cx.local.ccs811.set_environment(temperature.into(), humidity.into()).unwrap();


//          cx.local.tx.lock(|tx| writeln!(tx, "\rstart\r",)).unwrap();
//          for i in 0..*cx.local.index {
//              let data = cx.local.measurements[i];
//              let en = if i == 0 {
//                  (0.0, 0.0)
//              } else {
//                  cx.local.env[i - 1]
//              };
//              //writeln!(cx.local.tx,  "{},{},{},{},{},{:.2},{:.2}\r", i, data.eco2,
//              //        data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1 ).unwrap();
//              cx.local
//                  .tx
//                  .lock(|tx| {
//                      writeln!(
//                          tx,
//                          "{},{},{},{},{},{:.2},{:.2}\r",
//                          i, data.eco2, data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1
//                      )
//                  })
//                  .unwrap();
//          }
       }
    }

    #[task(shared = [led], priority=1 )]
    async fn blink(_cx: blink::Context, duration: u32) {
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        Mono::delay(duration.millis()).await;
        crate::app::led_off::spawn().unwrap();
    }

    #[task(shared = [led], priority=1 )]
    async fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], priority=1 )]
    async fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
