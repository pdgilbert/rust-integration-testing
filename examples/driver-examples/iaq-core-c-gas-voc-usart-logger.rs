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
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
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
    
    use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};

    #[cfg(feature = "stm32f0xx")]
    use stm32f0xx_hal::{
        pac::{Peripherals, USART1},
        prelude::*,
        serial::{Serial, Tx},
   };

    #[cfg(feature = "stm32f0xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL
   
    #[cfg(feature = "stm32f0xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c2, I2c2Type as I2cType,};

    #[cfg(feature = "stm32f0xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f0xx")]
    fn setup(mut dp: Peripherals) ->  (I2cType, LedType, TxType) {    
       let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
       let gpioa = dp.GPIOA.split(&mut rcc);

       let i2c = setup_i2c2(dp.I2C2, dp.GPIOB.split(&mut rcc),  &mut rcc);

       let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
       led.off();

       let (tx, rx) = cortex_m::interrupt::free(move |cs| {
           (
               gpioa.pa9.into_alternate_af1(cs),  //tx pa9
               gpioa.pa10.into_alternate_af1(cs), //rx pa10
           )
       });

       let (tx, _rx) = Serial::usart1(dp.USART1, (tx, rx), 9600.bps(), &mut rcc).split();

       (i2c, led, tx)
    }




    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        device::USART1,
        pac::Peripherals, 
        prelude::*,
        serial::{Config, Serial, Tx},
    };

    #[cfg(feature = "stm32f1xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f1xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        let gpiob = dp.GPIOB.split();

        //afio  needed for i2c1 (PB8, PB9) but not i2c2
        let i2c = setup_i2c1(dp.I2C1, gpiob, &mut afio, &clocks);

        let mut led = setup_led(dp.GPIOC.split()); 
        led.off();

        // NOTE, try to figure out the proper way to deal with this:
        // Using gpiob (PB6-7) for serial causes a move problem because gpiob is also used for i2c.
        // (There can also a move problem with afio if setup_i2c1() takes afio rather than &mut afio,
        // but that can be resolved by just doing serial() before setup_i2c1().)

        let mut gpioa = dp.GPIOA.split();
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
   

        (i2c, led, tx)
    }

    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        gpio::{PushPull, AF7,
            gpioa::PA9,
        },
        pac::{Peripherals, USART1},
        prelude::*,
        serial::{Serial, Tx},
    };

    #[cfg(feature = "stm32f3xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f3xx")]
    type TxType = Tx<USART1, PA9<AF7<PushPull>>>;

    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
       //fn setup(dp: Peripherals) -> (I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>, LedType, TxType ) {
       let mut flash = dp.FLASH.constrain();
       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze(&mut flash.acr);

       let gpiob = dp.GPIOB.split(&mut rcc.ahb);
       let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);

       let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
       led.off();

       let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
       let (tx, _rx) = Serial::new(
           dp.USART1,
           (
               gpioa
                   .pa9
                   .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
               gpioa
                   .pa10
                   .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
           ),
           115200.Bd(),
           clocks,
           &mut rcc.apb2,
       )
       .split();

       (i2c, led, tx)
    }

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        pac::{Peripherals, USART1}, //I2C1
        prelude::*,
        serial::{config::Config, Serial, Tx},
    };

    #[cfg(feature = "stm32f4xx")]
    const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f4xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f4xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
       let rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       let gpioa = dp.GPIOA.split();
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

       (i2c, led, tx)
    }

    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        pac,
        pac::{Peripherals, },
        prelude::*,
        serial::{Config, Oversampling, Serial, Tx},
    };

    #[cfg(feature = "stm32f7xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f7xx")]
    type TxType = Tx<pac::USART2>;

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();
       //let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

       let mut led = setup_led(dp.GPIOC.split());
       led.off();

       let gpioa = dp.GPIOA.split();

       let (tx, _rx) = Serial::new(
           dp.USART2,
           (
               gpioa.pa2.into_alternate(),
               gpioa.pa3.into_alternate(),
           ),
           clocks,
           Config {
               baud_rate: 9600.Bps(),
               oversampling: Oversampling::By16,
               character_match: None,
           },
       )
       .split();

       (i2c, led, tx)
    }

    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        pac::{Peripherals, USART2},
        prelude::*,
        serial::Tx,
    };

    #[cfg(feature = "stm32h7xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32h7xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32h7xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32h7xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
       let pwr = dp.PWR.constrain();
       let vos = pwr.freeze();
       let rcc = dp.RCC.constrain();
       let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
       let clocks = ccdr.clocks;

       let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
       let i2cx = ccdr.peripheral.I2C1;  //.I2C4;

       let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
       let mut led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
       led.off();

       let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

       let (tx, _rx) = dp
           .USART2
           .serial(
               (
                   gpioa.pa2.into_alternate_af7(),
                   gpioa.pa3.into_alternate_af7(),
               ),
               9600.bps(),
               ccdr.peripheral.USART2,
               &clocks,
           )
           .unwrap()
           .split();

       (i2c, led, tx)
    }

    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l0xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l0xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) ->(I2cType, LedType, TxType) {
       let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
       let clocks = rcc.clocks;

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), dp.AFIO.constrain(), &clocks);
       let mut led = setup_led(dp.GPIOC.split(&mut rcc));
       led.off();

       let (tx, _rx) = dp.USART1.usart(
            gpioa.pa9,
            gpioa.pa10,
            Config::default().baudrate(115200.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

        (i2c, led, tx)
    }

    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        prelude::*,
        rcc::Config as rccConfig,
        serial::{Config, SerialExt, Tx},
        stm32::{Peripherals, USART1},
    };

    #[cfg(feature = "stm32l1xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l1xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
       let mut rcc = dp.RCC.freeze(rccConfig::hsi());

       let gpiob = dp.GPIOB.split(&mut rcc);

// setup_i2c1 NOT WORKING
       let scl = gpiob.pb8.into_open_drain_output();
       let sda = gpiob.pb9.into_open_drain_output(); 
       let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);
//       let i2c = setup_i2c1(dp.I2C1, gpiob, rcc);

       let mut led = setup_led(gpiob.pb6);
       led.off();

       let gpioa = dp.GPIOA.split(&mut rcc);
       let (tx, _rx) = dp
           .USART1
           .usart(
               (gpioa.pa9, gpioa.pa10),
               Config::default().baudrate(9600.bps()),
               &mut rcc,
           )
           .unwrap()
           .split();

       (i2c, led, tx)
    }

    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        pac::{Peripherals, USART2},
        prelude::*,
        serial::{Config as serialConfig, Serial, Tx},
    };

    #[cfg(feature = "stm32l4xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l4xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l4xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, TxType) {
       let mut flash = dp.FLASH.constrain();
       let mut rcc = dp.RCC.constrain();
       let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
       let clocks = rcc
           .cfgr
           .sysclk(80.mhz())
           .pclk1(80.mhz())
           .pclk2(80.mhz())
           .freeze(&mut flash.acr, &mut pwr);

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
       let mut led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
       led.off();

       let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
 
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
           serialConfig::default().baudrate(9600.bps()),
           clocks,
           &mut rcc.apb1r1,
       )
       .split();

       (i2c, led, tx)
    }

    // End of hal/MCU specific setup. Following should be generic code.

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
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, MONOCLOCK);

        rtt_init_print!();
        rprintln!("iAQ-Core-C example");

        let device: Peripherals = cx.device;

        let (i2c, mut led, mut tx) = setup(device);

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
