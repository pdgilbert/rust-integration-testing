//!  Substantially modified for rtic 0.6
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

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_rtt_target as _;

use rtic::app;

#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    use cortex_m::asm; //asm::delay(N:u32) blocks the program for at least N CPU cycles.
                       //delay_ms could be used but needs to use a timer other than Systick
                       //use embedded_hal::blocking::delay; //delay::delay_ms(N:u32) blocks the program for N ms.

    use core::fmt::Write;
    use embedded_ccs811::{
        mode as Ccs811Mode, prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode,
        SlaveAddr as Ccs811SlaveAddr,
    };
    use hdc20xx::{mode as Hdc20xxMode, Hdc20xx, SlaveAddr as Hdc20xxSlaveAddr};
    use nb::block;
    use rtt_target::{rprintln, rtt_init_print};
    use shared_bus_rtic::SharedBus;
    use systick_monotonic::*;

    //const PERIOD: u32 = 1_000_000_000; // clock pulses for 10 seconds
    const PERIOD: u64 = 10;  // used as seconds
    //const PERIOD: Duration<T, NOM, DENOM> = 10.secs();

    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        device::USART1,
        gpio::{
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
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

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

        let mut gpioc = dp.GPIOC.split();
        //let mut led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
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

        (i2c, led, tx)
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

        (i2c, led, tx)
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

        (i2c, led, tx)
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

        (i2c, led, tx)
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

        (i2c, led, tx)
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

        (i2c, led, tx)
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

        (i2c, led, tx)
    }

    // End of hal/MCU specific setup. Following should be generic code.

    pub trait LED {
        // depending on board wiring, on may be set_high or set_low, with off also reversed
        // implementation should deal with this difference
        fn on(&mut self) -> ();
        fn off(&mut self) -> ();
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<CLOCK>;

    /*
     * shared-bus-rtic aggregate: multiple peripherals on a single i2c bus
     *
     * According to shared-bus-rtic docs:
     * Note that all of the drivers that use the same underlying bus **must** be stored within a single
     * resource (e.g. as one larger `struct`) within the RTIC resources. This ensures that RTIC will
     * prevent one driver from interrupting another while they are using the same underlying bus.
     */

    #[shared]
    struct Shared {
        led: LedType,
        ccs811: Ccs811Awake<SharedBus<I2cBus>, Ccs811Mode::App>,
        hdc2080: Hdc20xx<SharedBus<I2cBus>, Hdc20xxMode::OneShot>,
        tx: TxType,
    }

    #[local]
    struct Local {
        led_state: bool,
        env: [(f32, f32); 1200],
        index: usize,
        measurements: [AlgorithmResult; 1200],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, CLOCK);

        rtt_init_print!();
        rprintln!("CCS811/HDC2080 example");

        let device: Peripherals = cx.device;

        let (i2c, mut led, mut tx) = setup(device);

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

        let manager = shared_bus_rtic::new!(i2c, I2cBus);
        let mut hdc2080 = Hdc20xx::new(manager.acquire(), Hdc20xxSlaveAddr::default());
        let mut ccs811 = Ccs811Awake::new(manager.acquire(), Ccs811SlaveAddr::default());
        ccs811.software_reset().unwrap();

        // Delay while ccs811 resets.
        // Note that this delay cannot use SYST because Monotonics uses that (for spawn,
        //   although spawn has not yet happened so there may be a way?)

        led.on();
        asm::delay(3 * CLOCK); // (3 * CLOCK cycles give aprox 3 second delay
                               //delay::delay_ms(3_000_u16);
        led.off();
        led_state = false;

        let mut ccs811 = ccs811.start_application().ok().unwrap();
        let en = block!(hdc2080.read()).unwrap();
        ccs811
            .set_environment(en.temperature, en.humidity.unwrap_or(0.0))
            .unwrap();
        ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

        measure::spawn_after(PERIOD.secs()).unwrap();

        writeln!(tx, "start\r",).unwrap();

        (
            Shared {
                led: led,
                ccs811: ccs811,
                hdc2080: hdc2080,
                tx: tx,
            },
            Local {
                led_state: led_state,
                env: env,
                index: index,
                measurements: measurements,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [led, ccs811, hdc2080, tx], local = [led_state, env, index, measurements])]
    fn measure(mut cx: measure::Context) {
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
        measure::spawn_after(PERIOD.secs()).unwrap();
    }
}
