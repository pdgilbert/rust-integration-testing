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
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use embedded_ccs811::{
        mode as Ccs811Mode, prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode,
        SlaveAddr as Ccs811SlaveAddr,
    };
 
    //https://github.com/michaelbeaumont/dht-sensor
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::Reading;
    #[cfg(feature = "dht22")]
    use dht_sensor::dht22::Reading;
    use dht_sensor::*;

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

    use core::fmt::Write;
    use systick_monotonic::*;
    use nb::block;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs
    use fugit::TimerDuration;

    // set up for shared bus even though only one i2c device is used here
    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    const MONOTICK: u32 = 100;
    const READ_INTERVAL: u64 = 10;  // used as seconds
    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};

 

    #[cfg(feature = "stm32f0xx")]
    use stm32f0xx_hal::{
        gpio::{gpioa::PA8, OpenDrain, Output},
        pac::{Peripherals, USART1},
        prelude::*,
        serial::{Serial, Tx},
   };

    #[cfg(feature = "stm32f0xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f0xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32f0xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c2, I2c2Type as I2cType,};

    #[cfg(feature = "stm32f0xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f0xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32f0xx")]
    fn setup(mut dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {    
       let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
       let gpioa = dp.GPIOA.split(&mut rcc);

       let mut dht = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));
       dht.set_high().ok();

       let i2c = setup_i2c2(dp.I2C2, dp.GPIOB.split(&mut rcc),  &mut rcc);

       let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
       led.off();

       let delay = DelayType{};

       let (tx, rx) = cortex_m::interrupt::free(move |cs| {
           (
               gpioa.pa9.into_alternate_af1(cs),  //tx pa9
               gpioa.pa10.into_alternate_af1(cs), //rx pa10
           )
       });

       let (tx, _rx) = Serial::usart1(dp.USART1, (tx, rx), 9600.bps(), &mut rcc).split();

       (dht, i2c, led, tx, delay)
    }


    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        device::USART1,
        gpio::{OpenDrain, Output,
            gpioa::{PA8},
        },
        pac::Peripherals, //I2C1
        prelude::*,
        serial::{Config, Serial, Tx},
    };



    #[cfg(feature = "stm32f1xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32f1xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f1xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f1xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();
        let clocks = rcc
            .cfgr
            //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
            //.sysclk(72.mhz()) // system clock 8 MHz default, max 72MHz
            //.pclk1(36.mhz())  // system clock 8 MHz default, max 36MHz ?
            .freeze(&mut flash.acr);
        //let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

        hprintln!("hclk {:?}",   clocks.hclk()).unwrap();
        hprintln!("sysclk {:?}", clocks.sysclk()).unwrap();
        hprintln!("pclk1 {:?}",  clocks.pclk1()).unwrap();
        hprintln!("pclk2 {:?}",  clocks.pclk2()).unwrap();
        hprintln!("pclk1_tim {:?}", clocks.pclk1_tim()).unwrap();
        hprintln!("pclk2_tim {:?}", clocks.pclk2_tim()).unwrap();
        hprintln!("adcclk {:?}",    clocks.adcclk()).unwrap();
        //hprintln!("usbclk_valid {:?}", clocks.usbclk_valid()).unwrap(); not fo all MCUs

        let mut gpioa = dp.GPIOA.split();
        let dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);

        let gpiob = dp.GPIOB.split();

        //afio  needed for i2c1 (PB8, PB9) but not i2c2
        let i2c = setup_i2c1(dp.I2C1, gpiob, &mut afio, &clocks);

        let mut led = setup_led(dp.GPIOC.split()); 
        led.off();

        let delay = DelayType{};

        // NOTE, try to figure out the proper way to deal with this:
        // Using gpiob (PB6-7) for serial causes a move problem because gpiob is also used for i2c.
        // (There can also a move problem with afio if setup_i2c1() takes afio rather than &mut afio,
        // but that can be resolved by just doing serial() before setup_i2c1().)

        let (tx, _rx) = Serial::usart1(
           dp.USART1,
           // (gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl), 
           //  gpiob.pb7,
           (gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
            gpioa.pa10,
           ),
           &mut afio.mapr,
           Config::default().baudrate(115200.bps()), 
           clocks,
        ).split();
   
        (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        gpio::{
            gpioa::{PA8, PA9},
            OpenDrain, Output, PushPull, AF7,
        },
        pac::{Peripherals, USART1},
        prelude::*,
        serial::{Serial, Tx},
    };

    #[cfg(feature = "stm32f3xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32f3xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f3xx")]
    type TxType = Tx<USART1, PA9<AF7<PushPull>>>;
    //type TxType = Tx<USART1, impl TxPin<USART1>>;  // impl is unstable in type alias
    // See  https://github.com/stm32-rs/stm32f3xx-hal/issues/288
    //   regarding why it is necessary to specify the concrete pin here.

    #[cfg(feature = "stm32f3xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> (DhtPin, I2cType, LedType, TxType, DelayType) {
       let mut flash = dp.FLASH.constrain();
       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze(&mut flash.acr);
    
       let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
       let dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

       let gpiob = dp.GPIOB.split(&mut rcc.ahb);
       let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);

       let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
       led.off();

       let delay = DelayType{};

       let (tx, _rx) = Serial::new(
           dp.USART1,
           (
               gpioa
                   .pa9
                   .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
               gpioa
                   .pa10
                   .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
           ),
           115200.Bd(),
           clocks,
           &mut rcc.apb2,
       )
       .split();

       (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        timer::FDelay,
        gpio::{OpenDrain, Output,
               gpioa::PA8,
        },
        pac::{Peripherals, USART1, TIM2}, //I2C1
        prelude::*,
        serial::{config::Config, Serial, Tx},
    };

    #[cfg(feature = "stm32f4xx")]
    const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32f4xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f4xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f4xx")]
    pub type DelayType = FDelay<TIM2, 1000000_u32>;

    #[cfg(feature = "stm32f4xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
       let gpioa = dp.GPIOA.split();
       let dht = gpioa.pa8.into_open_drain_output();

       let rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       //let delay = DelayType{};
       let delay = dp.TIM2.delay_us(&clocks);

       let tx = gpioa.pa9.into_alternate();
       let rx = gpioa.pa10.into_alternate();
       let (tx, _rx) = Serial::new(
           dp.USART1,
           (tx, rx),
           Config::default().baudrate(115200.bps()),
           &clocks,
       )
       .unwrap()
       .split();

       (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        gpio::{Output, OpenDrain,
            gpioa::PA8,
        },
        pac,
        pac::{Peripherals, },
        prelude::*,
        serial::{Config, Oversampling, Serial, Tx},
    };

    #[cfg(feature = "stm32f7xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32f7xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f7xx")]
    type TxType = Tx<pac::USART2>;

    #[cfg(feature = "stm32f7xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
       let gpioa = dp.GPIOA.split();
       let dht   = gpioa .pa8.into_open_drain_output();

       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();
       //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

       let mut led = setup_led(dp.GPIOC.split());
       led.off();

       let delay = DelayType{};

       let (tx, _rx) = Serial::new(
           dp.USART2,
           (
               gpioa.pa2.into_alternate(),
               gpioa.pa3.into_alternate(),
           ),
           clocks,
           Config {
               baud_rate: 115200.Bps(), //should be bps. See https://github.com/stm32-rs/stm32f7xx-hal/issues/141
               oversampling: Oversampling::By16,
               character_match: None,
           },
       ).split();

      (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        gpio::{Output, OpenDrain,
               gpioa::PA8
        },
        pac::{Peripherals, USART2},
        prelude::*,
        serial::Tx,
    };

    #[cfg(feature = "stm32h7xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32h7xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32h7xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32h7xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32h7xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32h7xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32h7xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
       let pwr = dp.PWR.constrain();
       let vos = pwr.freeze();
       let rcc = dp.RCC.constrain();
       let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
       let clocks = ccdr.clocks;

       let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

       let dht = gpioa.pa8.into_open_drain_output();

       let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
       let i2cx = ccdr.peripheral.I2C1;  //.I2C4;

       let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
       let mut led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
       led.off();

       let delay = DelayType{};

       let (tx, _rx) = dp
           .USART2
           .serial(
               (
                   gpioa.pa2.into_alternate_af7(),
                   gpioa.pa3.into_alternate_af7(),
               ),
               115200.bps(),
               ccdr.peripheral.USART2,
               &clocks,
           )
           .unwrap()
           .split();

       (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull, OpenDrain, gpioa::PA8},
        pac::{Peripherals, USART1},
        prelude::*,
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l0xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l0xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32l0xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};
   
    #[cfg(feature = "stm32l0xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32l0xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
       // UNTESTED
       let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
       let clocks = rcc.clocks;

       let mut dht = dp.GPIOA.split(&mut rcc).pa8.into_open_drain_output();
 
       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), dp.AFIO.constrain(), &clocks);
       let mut led = setup_led(dp.GPIOC.split(&mut rcc));
       led.off();

       let delay = DelayType{};

       let (tx, _rx) = dp.USART1.usart(
            gpioa.pa9,
            gpioa.pa10,
            Config::default().baudrate(115200.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

       (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        gpio::{OpenDrain, Output,
               gpioa::PA8,
        },
        prelude::*,
        rcc::Config as rccConfig,
        serial::{Config, SerialExt, Tx},
        stm32::{Peripherals, USART1},
        //serial::{Config, Rx, Serial1Ext, Serial2Ext, Serial4Ext, Tx},
    };

    #[cfg(feature = "stm32l1xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32l1xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32l1xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l1xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32l1xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
       let mut rcc = dp.RCC.freeze(rccConfig::hsi());

       let gpioa = dp.GPIOA.split(&mut rcc);
       let dht = gpioa.pa8.into_open_drain_output();

       let gpiob = dp.GPIOB.split(&mut rcc);

// setup_i2c1 NOT WORKING
       let scl = gpiob.pb8.into_open_drain_output();
       let sda = gpiob.pb9.into_open_drain_output(); 
       let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);
//       let i2c = setup_i2c1(dp.I2C1, gpiob, rcc);

       let mut led = setup_led(gpiob.pb6);
       led.off();

       let delay = DelayType{};


       let (tx, _rx) = dp
           .USART1
           .usart(
               (gpioa.pa9, gpioa.pa10),
               Config::default().baudrate(115200.bps()),
               &mut rcc,
           )
           .unwrap()
           .split();

       (dht, i2c, led, tx, delay)
    }



    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        gpio::{OpenDrain, Output,
            gpioa::PA8,
        },
        pac::{Peripherals, USART2},
        prelude::*,
        serial::{Config as serialConfig, Serial, Tx},
    };

    #[cfg(feature = "stm32l4xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l4xx")]
    type DhtPin = PA8<Output<OpenDrain>>;
   
    #[cfg(feature = "stm32l4xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l4xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32l4xx")]
    use rust_integration_testing_of_examples::alt_delay::{AltDelay as DelayType};

    #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) ->  (DhtPin, I2cType, LedType, TxType, DelayType) {
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
        let clocks = rcc
            .cfgr
            .sysclk(80.mhz())
            .pclk1(80.mhz())
            .pclk2(80.mhz())
            .freeze(&mut flash.acr, &mut pwr);

       let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
       let dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
       let mut led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
       led.off();

       let delay = DelayType{};

       let (tx, _rx) = Serial::usart2(
           dp.USART2,
           (
               gpioa
                   .pa2
                   .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
               gpioa
                   .pa3
                   .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
           ),
           serialConfig::default().baudrate(115200.bps()),
           clocks,
           &mut rcc.apb1r1,
       )
       .split();

       (dht, i2c, led, tx, delay)
    }

    // End of hal/MCU specific setup. Following should be generic code.


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, MONOCLOCK);

        //rtt_init_print!();
        //rprintln!("CCS811 example");

        let (mut dht, i2c, mut led, mut tx, mut delay) = setup(cx.device);
   
        led.on(); 
        delay.delay_ms(1000u32);
        led.off();

        // 5 sec systick timer  check
        // beware this needs to happen before other spawn activity interferes with led.
        //led_on::spawn_after(1.secs()).unwrap(); 
        //led_off::spawn_after(6.secs()).unwrap();

        dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
        delay.delay_ms(2000_u32); //  2 second delay for dhtsensor initialization
        
        // intial dht reading
        let (temperature, humidity) = match Reading::read(&mut delay, &mut dht) {
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

        // initialize ccs811
        //let env: [(f32, f32); 1200] = [(0.0, 0.0); 1200];
        //let index: usize = 0;
        //let measurements: [AlgorithmResult; 1200] = [AlgorithmResult {
        //    eco2: 0, etvoc: 0, raw_current: 0, raw_voltage: 0, }; 1200];

       let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

//    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
//    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
//        .into_buffered_graphics_mode();

        let mut ccs811 = Ccs811Awake::new(manager.acquire_i2c(), Ccs811SlaveAddr::default());
        hprintln!("let mut ccs811 = Ccs811Awake::new(").unwrap();
        ccs811.software_reset().unwrap();
        hprintln!("_reset").unwrap();

        delay.delay_ms(3000u32);  // Delay while ccs811 resets
        hprintln!("delay.delay_ms(3000u32)").unwrap();

        let mut ccs811 = ccs811.start_application().ok().unwrap();
        hprintln!("let mut ccs811 = ccs811.start_application(").unwrap();
        ccs811.set_environment(temperature.into(), humidity.into()).unwrap(); //i8 into f32, u8 into f32
        ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();
        hprintln!("ccs811.set_mode(3000u32)").unwrap();

        // make certain this does not start sooner than end of systick timer led check above
        measure::spawn_after(READ_INTERVAL.secs()).unwrap();

        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();
        writeln!(tx, "start\r",).unwrap();

        (Shared {led}, Local {dht, ccs811, tx, delay,}, init::Monotonics(mono))
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
        dht: DhtPin,
        ccs811: Ccs811Awake<I2cProxy<'static,   Mutex<RefCell<I2cType>>>, Ccs811Mode::App>,
        tx: TxType,
        delay:DelayType,
    }

    #[idle(local = [])]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). It may affect debugging.
           rtic::export::wfi()
        }
    }

    //#[task(shared = [dht, ccs811, led, tx, delay], local = [env, index, measurements], capacity=5)]
    #[task(shared = [led,], local = [dht, ccs811, delay, tx,], capacity=2)]
    fn measure(cx: measure::Context) {
        //hprintln!("measure").unwrap();
        blink::spawn(BLINK_DURATION.millis()).ok();

        // this might be nicer if read could be done by spawn rather than wait for delay
        let delay = cx.local.delay;
        let dht = cx.local.dht;
        let z = Reading::read(delay, dht);
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


//       cx.local.tx.lock(|tx| writeln!(tx, "\rstart\r",)).unwrap();
//       for i in 0..*cx.local.index {
//           let data = cx.local.measurements[i];
//           let en = if i == 0 {
//               (0.0, 0.0)
//           } else {
//               cx.local.env[i - 1]
//           };
//           //writeln!(cx.local.tx,  "{},{},{},{},{},{:.2},{:.2}\r", i, data.eco2,
//           //        data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1 ).unwrap();
//           cx.local
//               .tx
//               .lock(|tx| {
//                   writeln!(
//                       tx,
//                       "{},{},{},{},{},{:.2},{:.2}\r",
//                       i, data.eco2, data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1
//                   )
//               })
//               .unwrap();
//       }
        measure::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        crate::app::led_off::spawn_after(duration).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], capacity=2)]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
