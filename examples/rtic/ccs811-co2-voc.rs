//!  CLEANUP DESCRIPTION. DISPLAY OR LOG???
//! This example is derived from driver-examples/ccs811-gas-voc-usart-logger.rs
//! which comes from examples by Diego Barrios Romero.
//! See the introductory blog post with some pictures here:
//! https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
//!
//! Below has been substantially modified for rtic 1.0.0 and to run with various HALs,
//! and to use a DHT11 sensor to compensate for the ambient temperature and humidity,
//! and to remove some logging.
//! (See driver-examples/ccs811-gas-voc-usart-logger.rs for hdc2080
//!   temperature and humidity sensor which uses i2c. That example also keeps logging info.)

//! Continuously measure the eCO2 and eTVOC in the air, logs the values and sends
//! them through the serial interface every 10 seconds.
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

#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
//#[#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

    use core::fmt::Write;

    use systick_monotonic::*;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs

    use fugit::TimerDuration;

    const MONOTICK: u32 = 100;
    const READ_INTERVAL: u64 = 10;  // used as seconds
    const BLINK_DURATION: u64 = 20;  // used as milliseconds


    //https://github.com/michaelbeaumont/dht-sensor
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::Reading;
    #[cfg(feature = "dht22")]
    use dht_sensor::dht22::Reading;
    use dht_sensor::*;

    // set up for shared bus even though only one i2c device is used here
    use shared_bus_rtic::SharedBus;

    use embedded_ccs811::{
        mode as Ccs811Mode, prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode,
        SlaveAddr as Ccs811SlaveAddr,
    };
 
    use nb::block;
    
    //use rust_integration_testing_of_examples::i2c_led_delay::{setup_led, LED};

    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        device::USART1,
        gpio::{
            gpioa::{PA8},
            gpiob::{PB8, PB9},
            gpioc::PC13,
            Alternate,
            OpenDrain,
            Output,
            PushPull, //State,
        },
        i2c::{BlockingI2c, Mode}, //Pins
        pac,
        pac::Peripherals, //I2C1
        prelude::*,
        serial::{Config, Serial, Tx},
    };

    #[cfg(feature = "stm32f1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    type LedType = PC13<Output<PushPull>>;
    //impl LED, Delay

    #[cfg(feature = "stm32f1xx")]
    type I2cBus = BlockingI2c<pac::I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;
    //BlockingI2c<I2C1, impl Pins<I2C1>>

    #[cfg(feature = "stm32f1xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) -> (PA8<Output<OpenDrain>>, I2cBus, LedType, TxType, AltDelay) {
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();
        let clocks = rcc
            .cfgr
            //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
            //.sysclk(72.mhz()) // system clock 8 MHz default, max 72MHz
            //.pclk1(36.mhz())  // system clock 8 MHz default, max 36MHz ?
            .freeze(&mut flash.acr);

        hprintln!("hclk {:?}",   clocks.hclk()).unwrap();
        hprintln!("sysclk {:?}", clocks.sysclk()).unwrap();
        hprintln!("pclk1 {:?}",  clocks.pclk1()).unwrap();
        hprintln!("pclk2 {:?}",  clocks.pclk2()).unwrap();
        hprintln!("pclk1_tim {:?}", clocks.pclk1_tim()).unwrap();
        hprintln!("pclk2_tim {:?}", clocks.pclk2_tim()).unwrap();
        hprintln!("adcclk {:?}",    clocks.adcclk()).unwrap();
        hprintln!("usbclk_valid {:?}", clocks.usbclk_valid()).unwrap();

        let mut gpioa = dp.GPIOA.split();

        let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
        dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

        let mut delay = AltDelay{};
        delay.delay_ms(2000u32);  // 2 second for sensor to initialize

        let mut gpiob = dp.GPIOB.split();

        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Standard {
                frequency: 100_000.hz(),
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );
        let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let rx = gpiob.pb7;
        let serial = Serial::usart1(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            clocks,
        );
        let (tx, _rx) = serial.split();

        //let led = setup_led(dp.GPIOC.split());
        let mut gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_low()
            }
            fn off(&mut self) -> () {
                self.set_high()
            }
        }

        led.off();
        
        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        gpio::{
            gpioa::PA9,
            gpiob::{PB6, PB7},
            gpioe::PE9,
            OpenDrain, Output, PushPull, AF4, AF7,
        },
        i2c::I2c,
        pac::{Peripherals, I2C1, USART1},
        prelude::*,
        serial::{Serial, Tx},
    };

    #[cfg(feature = "stm32f3xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    type LedType = PE9<Output<PushPull>>;

    #[cfg(feature = "stm32f3xx")]
    type I2cBus = I2c<I2C1, (PB6<AF4<OpenDrain>>, PB7<AF4<OpenDrain>>)>;
    //or type I2cBus = I2c<I2C1, (PB6<Alternate<OpenDrain, { 4_u8 }>>, PB7<Alternate<OpenDrain, { 4_u8 }>>)>;

    #[cfg(feature = "stm32f3xx")]
    type TxType = Tx<USART1, PA9<AF7<PushPull>>>;
    //type TxType = Tx<USART1, impl TxPin<USART1>>;  // impl is unstable in type alias
    // See  https://github.com/stm32-rs/stm32f3xx-hal/issues/288
    //   regarding why it is necessary to specify the concrete pin here.

    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
        //fn setup(dp: Peripherals) -> (I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>, LedType, TxType ) {
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
        let scl =
            gpiob
                .pb6
                .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let sda =
            gpiob
                .pb7
                .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);

        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
        let (tx, _rx) = Serial::new(
            dp.USART1,
            (
                gpioa
                    .pa9
                    .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
                gpioa
                    .pa10
                    .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
            ),
            115200.Bd(),
            clocks,
            &mut rcc.apb2,
        )
        .split();

        let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
        let mut led = gpioe
            .pe9
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

        impl LED for PE9<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_high().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_low().unwrap()
            }
        }
        led.off();

        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        gpio::{
            gpiob::{PB8, PB9},
            gpioc::PC13,
            Alternate, OpenDrain, Output, PushPull,
        },
        i2c::I2c, //Pins Mode
        pac,
        pac::{Peripherals, USART1}, //I2C1
        prelude::*,
        serial::{config::Config, Serial, Tx},
    };

    #[cfg(feature = "stm32f4xx")]
    const CLOCK: u32 = 16_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    type LedType = PC13<Output<PushPull>>;
    //impl LED

    #[cfg(feature = "stm32f4xx")]
    type I2cBus = I2c<pac::I2C1, (PB8<Alternate<OpenDrain, 4u8>>, PB9<Alternate<OpenDrain, 4u8>>)>; //NO BlockingI2c
                                                                                  //BlockingI2c<I2C1, impl Pins<I2C1>>

    #[cfg(feature = "stm32f4xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32f4xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        let gpiob = dp.GPIOB.split();

        let scl = gpiob.pb8.into_alternate().set_open_drain();
        let sda = gpiob.pb9.into_alternate().set_open_drain();

        let i2c = I2c::new(dp.I2C1, (scl, sda), 100.khz(), &clocks);

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

        let gpioc = dp.GPIOC.split();
        //let mut led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
        let led = gpioc.pc13.into_push_pull_output();

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_low()
            }
            fn off(&mut self) -> () {
                self.set_high()
            }
        }

        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        gpio::{
            gpiob::{PB8, PB9},
            gpioc::PC13,
            AlternateOD, Output, PushPull, AF4,
        },
        i2c::{BlockingI2c, Mode, PinScl, PinSda},
        pac,
        pac::{Peripherals,  I2C1, USART2},
        prelude::*,
        serial::{Config, Oversampling, Serial, Tx},
    };

    #[cfg(feature = "stm32f7xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    type LedType = PC13<Output<PushPull>>;

    #[cfg(feature = "stm32f7xx")]
    type I2cBus = BlockingI2c<pac::I2C1, PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>>;
    //type I2cBus = BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>;

    #[cfg(feature = "stm32f7xx")]
    type TxType = Tx<pac::USART2>;

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
        //let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();
        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

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

        let gpiob = dp.GPIOB.split();

        let scl = gpiob.pb8.into_alternate_open_drain(); // scl on PB8
        let sda = gpiob.pb9.into_alternate_open_drain(); // sda on PB9

        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            Mode::standard(400_000.Hz()),
            clocks,
            &mut rcc.apb1,
            1000,
        );

        let gpioc = dp.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output();

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_low()
            }
            fn off(&mut self) -> () {
                self.set_high()
            }
        }

        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        i2c::I2c,
        pac::{Peripherals, I2C1, USART2},
        prelude::*,
        serial::Tx,
    };

    #[cfg(feature = "stm32h7xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32h7xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32h7xx")]
    type LedType = PC13<Output<PushPull>>;

    #[cfg(feature = "stm32h7xx")]
    type I2cBus = I2c<I2C1>;

    #[cfg(feature = "stm32h7xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32h7xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
        let clocks = ccdr.clocks;

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

        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

        let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain(); // sda on PB9

        let i2c = dp
            .I2C1
            .i2c((scl, sda), 400.khz(), ccdr.peripheral.I2C1, &clocks);

        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let led = gpioc.pc13.into_push_pull_output();

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_low().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_high().unwrap()
            }
        }

        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l0xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l0xx")]
    type LedType = PC13<Output<PushPull>>;

    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
        let gpioc = p.GPIOC.split(&mut rcc);
        let led = gpioc.pc13.into_push_pull_output();

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_low().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_high().unwrap()
            }
        }

        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        gpio::{
            gpiob::{PB6, PB8, PB9},
            OpenDrain, Output, PushPull,
        },
        i2c::{I2c,},
        prelude::*,
        rcc::Config as rccConfig,
        serial::{Config, SerialExt, Tx},
        stm32::{Peripherals, I2C1, USART1},
    };

    #[cfg(feature = "stm32l1xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32l1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    type LedType = PB6<Output<PushPull>>;

    #[cfg(feature = "stm32l1xx")]
    type I2cBus = I2c<I2C1, (PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>)>;

    #[cfg(feature = "stm32l1xx")]
    type TxType = Tx<USART1>;

    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
        let mut rcc = dp.RCC.freeze(rccConfig::hsi());

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

        let gpiob = dp.GPIOB.split(&mut rcc);

        let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
        let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

        let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

        let led = gpiob.pb6.into_push_pull_output();

        impl LED for PB6<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_high().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_low().unwrap()
            }
        }

        (dht, i2c, led, tx, delay)
    }

    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        gpio::{
            gpioa::{PA10, PA9},
            gpioc::PC13,
            Alternate, OpenDrain, Output, PushPull,
        },
        i2c::{Config as i2cConfig, I2c},
        pac::{Peripherals, I2C1, USART2},
        prelude::*,
        serial::{Config as serialConfig, Serial, Tx},
    };

    #[cfg(feature = "stm32l4xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l4xx")]
    type LedType = PC13<Output<PushPull>>;

//    #[cfg(feature = "stm32l4xx")]
//    type I2cBus = I2c<I2C1, pins(PA9, PA10)>;

    #[cfg(feature = "stm32l4xx")]
    type I2cBus = I2c<
        I2C1,
        (
            PA9 <Alternate<OpenDrain, 4u8>>,   
            PA10<Alternate<OpenDrain, 4u8>>,
        ),
    >;

    #[cfg(feature = "stm32l4xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
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
        //         let mut gpiob  = p.GPIOB.split(&mut rcc.ahb2);

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

        // following github.com/stm32-rs/stm32l4xx-hal/blob/master/examples/i2c_write.rs

        let mut scl =
            gpioa
                .pa9
                .into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // scl on PA9
        scl.internal_pull_up(&mut gpioa.pupdr, true);

        let mut sda =
            gpioa
                .pa10
                .into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // sda on PA10
        sda.internal_pull_up(&mut gpioa.pupdr, true);

        let i2c = I2c::i2c1(
            dp.I2C1,
            (scl, sda),
            i2cConfig::new(400.khz(), clocks),
            &mut rcc.apb1r1,
        );

        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
        let led = gpioc
            .pc13
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_low()
            }
            fn off(&mut self) -> () {
                self.set_high()
            }
        }

        (dht, i2c, led, tx, delay)
    }

    // End of hal/MCU specific setup. Following should be generic code.

    pub trait LED {
        // depending on board wiring, on may be set_high or set_low, with off also reversed
        // implementation should deal with this difference
        fn on(&mut self) -> ();
        fn off(&mut self) -> ();
    }

    use embedded_hal::blocking::delay::{DelayMs, DelayUs};

    // A delay is used in sensor (dht) initialization and read. 
    // Systick is used by monotonic (for spawn), so delay needs to use a timer other than Systick
    // asm::delay is not an accurate timer but gives a delay at least number of indicated clock cycles.

    use cortex_m::asm::delay; // argment in clock cycles so (5 * CLOCK) cycles gives aprox 5 second delay

    pub struct AltDelay {}

    impl DelayUs<u8> for AltDelay {
        fn delay_us(&mut self, t:u8) {
            delay((t as u32) * (CLOCK / 1_000_000)); 
        }
    }

    impl DelayUs<u32> for AltDelay {
        fn delay_us(&mut self, t:u32) {
            delay((t as u32) * (CLOCK / 1_000_000)); 
        }
    }

    impl DelayMs<u8> for AltDelay {
        fn delay_ms(&mut self, ms: u8) {
            delay((ms as u32) * (CLOCK / 1000)); 
        }
    }

    impl DelayMs<u32> for AltDelay {
        fn delay_ms(&mut self, ms: u32) {
            delay((ms as u32) * (CLOCK / 1000)); 
        }
    }


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, CLOCK);

        //rtt_init_print!();
        //rprintln!("CCS811 example");

        let (mut dht, i2c, mut led, mut tx, mut delay) = setup(cx.device);
   
        // 5 sec alt delay check
        led.on(); 
        delay.delay_ms(5000u32);  // 5 sec alt delay check
        led.off();

        // 5 sec systick timer  check
        // beware this needs to happen before other spawn activity interferes with led.
        led_on::spawn_after(1.secs()).unwrap(); 
        led_off::spawn_after(6.secs()).unwrap();

        // check dht
        let (temperature, humidity) = match Reading::read(&mut delay, &mut dht) {
            Ok(Reading{temperature, relative_humidity,}) 
               => {hprintln!("dht initialized. {} deg C, {}% RH", temperature, relative_humidity).unwrap();
                   (temperature, relative_humidity)
                  },
            Err(e) 
               =>  {hprintln!("dht initialize Error {:?}. Using default.", e).unwrap(); 
                    //panic!("Error reading DHT"),
                    (25, 40)  //supply default values
                   },
        };

        // initialize ccs811
        //let env: [(f32, f32); 1200] = [(0.0, 0.0); 1200];
        //let index: usize = 0;
        //let measurements: [AlgorithmResult; 1200] = [AlgorithmResult {
        //    eco2: 0, etvoc: 0, raw_current: 0, raw_voltage: 0, }; 1200];

        let manager = shared_bus_rtic::new!(i2c, I2cBus);

        let mut ccs811 = Ccs811Awake::new(manager.acquire(), Ccs811SlaveAddr::default());
        ccs811.software_reset().unwrap();

        delay.delay_ms(3000u32);  // Delay while ccs811 resets

        let mut ccs811 = ccs811.start_application().ok().unwrap();
        ccs811.set_environment(temperature.into(), humidity.into()).unwrap(); //i8 into f32, u8 into f32
        ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

        // make certain this does not start sooner than end of systick timer led check above
        measure::spawn_after(READ_INTERVAL.secs()).unwrap();

        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();
        writeln!(tx, "start\r",).unwrap();

        (Shared {dht, ccs811, led, tx, delay, }, 
         Local {}, 
         init::Monotonics(mono))
    }

    #[shared]
    struct Shared {
        dht: PA8<Output<OpenDrain>>,
        ccs811: Ccs811Awake<SharedBus<I2cBus>, Ccs811Mode::App>,
        led: LedType,
        tx: TxType,
        delay: AltDelay,
        //env: [(f32, f32); 1200],
        //index: usize,
        //measurements: [AlgorithmResult; 1200],
    }

    #[local]
    struct Local {
    }

    //#[task(shared = [dht, ccs811, led, tx, delay], local = [env, index, measurements], capacity=5)]
    #[task(shared = [dht, ccs811, led, tx, delay, ], capacity=5)]
    fn measure(mut cx: measure::Context) {
        //hprintln!("measure").unwrap();
        blink::spawn(BLINK_DURATION.millis()).ok();

        // this might be nicer if read could be done by spawn rather than wait for delay
        let delay = cx.shared.delay;
        let dht = cx.shared.dht;
        let z = (delay, dht).lock(|delay, dht| { Reading::read(delay, dht) });
        let (temperature, humidity) = match z {
            Ok(Reading {temperature, relative_humidity,})
               => (temperature, relative_humidity),
            Err(e) 
               =>  {hprintln!("dht Error {:?}. Using default.", e).unwrap(); 
                    //panic!("Error reading DHT"),
                    (25, 40)},  //supply default values
        };

        let data = cx.shared.ccs811.lock(|ccs811| block!(ccs811.data()))
                         .unwrap_or(AlgorithmResult::default());

        hprintln!("ccs811 data eco2:{}, etvoc:{}, raw_current:{}, raw_volt:{}", 
                          data.eco2, data.etvoc, data.raw_current, data.raw_voltage).unwrap();
        cx.shared.ccs811.lock(|ccs811| ccs811.set_environment(temperature.into(), humidity.into())).unwrap();


//       cx.shared.tx.lock(|tx| writeln!(tx, "\rstart\r",)).unwrap();
//       for i in 0..*cx.local.index {
//           let data = cx.local.measurements[i];
//           let en = if i == 0 {
//               (0.0, 0.0)
//           } else {
//               cx.local.env[i - 1]
//           };
//           //writeln!(cx.shared.tx,  "{},{},{},{},{},{:.2},{:.2}\r", i, data.eco2,
//           //        data.etvoc, data.raw_current, data.raw_voltage, en.0, en.1 ).unwrap();
//           cx.shared
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

    #[task(shared = [led], capacity=5)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        crate::app::led_off::spawn_after(duration).unwrap();
    }

    #[task(shared = [led], capacity=5)]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], capacity=5)]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
