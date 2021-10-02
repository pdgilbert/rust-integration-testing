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

#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))] 
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))] 
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]


mod app {
 
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::{Seconds, };

    use core::fmt::Write;
    use iaq_core::{IaqCore, Measurement};
    use nb::block;
    use rtt_target::{rprintln, rtt_init_print};
     
    //const PERIOD: u32 = 1_000_000_000; // 10 seconds
    const PERIOD: Seconds = Seconds(10);

    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        device::USART1,
        gpio::{
            gpiob::{PB8, PB9},
            gpioc::PC13,
            Alternate, OpenDrain, Output, PushPull, //State,
        },
        i2c::{BlockingI2c, Mode},
        pac,
        pac::Peripherals, //I2C1
        prelude::*,
        serial::{Config, Serial, Tx},
    };
    
    #[cfg(feature = "stm32f1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    type LedType = PC13<Output<PushPull>>;
    //impl LED
    
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
            gpiob::{PB6, PB7},
            gpioe::PE9,
            Alternate, OpenDrain,
            Otyper::Otype,
            Output, PushPull, AF4,
        },
        i2c::{I2c, SclPin, SdaPin},
        pac,
        pac::{Peripherals, I2C1, USART1},
        prelude::*,
        serial::{RxPin, Serial, Tx, TxPin},
    };
           
    #[cfg(feature = "stm32f3xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    type LedType = PE9<Output<PushPull>>;
    
    #[cfg(feature = "stm32f3xx")]
    type I2cBus = I2c<pac::I2C1, (PB6<Alternate<Otype, AF4>>, PB7<Alternate<Otype, AF4>>)>;
    //type I2cBus = I2c<pac::I2C1, (PB6<Alternate<OpenDrain, <AF4>>>, PB7<Alternate<OpenDrain, <AF4>>>)>;
    //type I2cBus = I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>;
    
    #[cfg(feature = "stm32f3xx")]
    type TxType = Tx<USART1, TxPin<USART1>>;
 
    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType ) {
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
    
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob
            .pb6
            .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let sda = gpiob
            .pb7
            .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);
    
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
        let (tx, _rx) = Serial::usart1(
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
        let led = gpioe
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
            AlternateOD, Output, PushPull, AF4,
        },
        i2c::I2c, //Pins Mode
        pac,
        pac::{Peripherals, USART1}, //I2C1
        prelude::*,
        serial::{config::Config, Serial, Tx},
    };
           
    #[cfg(feature = "stm32f4xx")]
    const CLOCK: u32 = 16_000_000;  //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    type LedType = PC13<Output<PushPull>>;
    //impl LED
    
    #[cfg(feature = "stm32f4xx")]
    type I2cBus = I2c<pac::I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>; //NO BlockingI2c
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
    
        let i2c = I2c::new(dp.I2C1, (scl, sda), 100.khz(), clocks);
    
        let gpioa = dp.GPIOA.split();
        let tx = gpioa.pa9.into_alternate();
        let rx = gpioa.pa10.into_alternate();
        let (tx, _rx) = Serial::new(
            dp.USART1,
            (tx, rx),
            Config::default().baudrate(115200.bps()),
            clocks,
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
        //led.off();   NEED TO FIX
    
        (i2c, led, tx)
    }
    
     
    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        gpio::{Output, PushPull, Alternate, AF4,
               gpioc::PC13, 
               gpiob::{PB8, PB9},
               },
        i2c::{BlockingI2c, Mode},
        pac::{Peripherals, USART2},
        pac,
        prelude::*,
        serial::{Config, Oversampling, Serial, Tx},
    };

    #[cfg(feature = "stm32f7xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL
    
    #[cfg(feature = "stm32f7xx")]
    type LedType = PC13<Output<PushPull>>;
        
    #[cfg(feature = "stm32f7xx")]
    type I2cBus = BlockingI2c<pac::I2C1, PB8<Alternate<AF4>>, PB9<Alternate<AF4>>>;
    //type I2cBus = BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>;

    #[cfg(feature = "stm32f7xx")]
    type TxType = Tx<USART2>;

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {

        //let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();
        let mut rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

        let gpioa = dp.GPIOA.split();

        let (tx, _rx) = Serial::new(
            dp.USART2,
            (
                gpioa.pa2.into_alternate_af7(),
                gpioa.pa3.into_alternate_af7(),
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

        let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain(); // sda on PB9

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
                self.set_low().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_high().unwrap()
            }
        }
    
        (i2c, led, tx)
    }
    
    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        gpio::{Output, PushPull,
               gpioc::PC13,},
        i2c::I2c,
        pac::{Peripherals, I2C1, USART2},
        prelude::*,
        serial::{Tx},
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

        let i2c =   dp.I2C1.i2c((scl, sda), 400.khz(), ccdr.peripheral.I2C1, &clocks);

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
        serial::{Config, Serial2Ext, Tx},
    };

    #[cfg(feature = "stm32l0xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL
    
    #[cfg(feature = "stm32l0xx")]
    type LedType = PC13<Output<PushPull>>;
            
    #[cfg(feature = "stm32l0xx")]
    type I2cBus = BlockingI2c<pac::I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;

    #[cfg(feature = "stm32l0xx")]
    type TxType = Tx<pac::USART1>;

    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) -> (I2cBus, LedType, TxType) {

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
        gpio::{Output, PushPull, OpenDrain,
               gpiob::{PB6, PB8, PB9},},
        i2c::{I2c},
        prelude::*,
        rcc::Config as rccConfig,
        stm32::{Peripherals, I2C1, USART1},
        serial::{Config, SerialExt, Tx},
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

        let gpioa = dp.GPIOA.split();

        let (tx, _rx) = dp
            .USART1
            .usart(
                (
                    gpioa.pa9,
                    gpioa.pa10,
                ), 
                Config::default().baudrate(9600.bps()),
                &mut rcc,
            )
            .unwrap()
            .split();

        let gpiob = dp.GPIOB.split();

        let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
        let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

        let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

        let gpiob = dp.GPIOB.split();
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
        gpio::{Output, PushPull, Alternate, AF4, OpenDrain,
               gpioa::{PA9, PA10},
               gpioc::PC13, },
        i2c::{I2c, Config as i2cConfig},
        pac::{Peripherals, I2C1, USART2},
        prelude::*,
        serial::{Serial, Tx, Config as serialConfig, },
    };
    
    #[cfg(feature = "stm32l4xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32l4xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL
  
    #[cfg(feature = "stm32l4xx")]
    type LedType = PC13<Output<PushPull>>;
            
    #[cfg(feature = "stm32l4xx")]
    type I2cBus = I2c<I2C1, (PA9<Alternate<AF4, OpenDrain>>, PA10<Alternate<AF4, OpenDrain>>)>;

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
                gpioa.pa2.into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
                gpioa.pa3.into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
            ),
            serialConfig::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1r1,
        )
        .split();

        // following github.com/stm32-rs/stm32l4xx-hal/blob/master/examples/i2c_write.rs

        let mut scl = gpioa
            .pa9
            .into_af4_opendrain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // scl on PA9
        scl.internal_pull_up(&mut gpioa.pupdr, true);

        let mut sda = gpioa
            .pa10
            .into_af4_opendrain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // sda on PA10
        sda.internal_pull_up(&mut gpioa.pupdr, true);

        let i2c = I2c::i2c1(dp.I2C1, (scl, sda), i2cConfig::new(400.khz(), clocks), &mut rcc.apb1r1);

        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    
        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self)  -> () { self.set_low().unwrap() }
            fn off(&mut self) -> () { self.set_high().unwrap()}
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
    type DwtMono = DwtSystick<CLOCK>;
    
    #[shared]
    struct Shared {
        led: LedType,
        sensor: IaqCore<I2cBus>,
        tx: TxType,
    }
   
    #[local]
    struct Local {
       led_state: bool, 
       index: usize,
       measurements: [Measurement; 2400],
    }

   
    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, CLOCK);

        rtt_init_print!();
        rprintln!("iAQ-Core-C example");
    
        let device: Peripherals = cx.device;
    
        let (i2c, mut led, mut tx) = setup(device);
    
        led.off();

        // Previously these were initialized static mut in fn measure()
        let led_state: bool = false;
        let index: usize = 0;
        let measurements:[Measurement; 2400]  = [Measurement { co2: 0, tvoc: 0, resistance: 0, }; 2400];

        measure::spawn_after(PERIOD).unwrap();

        let sensor = IaqCore::new(i2c);

        writeln!(tx, "start\r",).unwrap();

        (Shared {led: led, sensor: sensor, tx: tx }, 
         Local {led_state: led_state,  index: index,  measurements: measurements}, 
         init::Monotonics(mono))
    }
    
 
    #[task(shared = [led, sensor, tx], local = [led_state, index, measurements])]
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
            let data = cx.shared.sensor.lock(|sensor| block!(sensor.data())).unwrap_or(default);
            cx.local.measurements[*cx.local.index] = data;
            *cx.local.index += 1;
        }
        for i in 0..*cx.local.index {
            let data = cx.local.measurements[i];
            //writeln!(cx.resources.tx,"{},{},{},{}\r",i, data.co2, data.tvoc, data.resistance).unwrap();
            cx.shared.tx.lock(|tx| writeln!(tx, "{},{},{},{}\r",
                                   i, data.co2, data.tvoc, data.resistance)).unwrap();
        }
        measure::spawn_after(PERIOD).unwrap();
    }

}
