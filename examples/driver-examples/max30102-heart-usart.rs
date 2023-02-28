//! Continuously read the heart data and send it through USART.
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1 and USART1.
//!
//! To setup the serial communication, have a look at the discovery book:
//! https://rust-embedded.github.io/discovery/10-serial-communication/index.html
//! (It is 9600 baud standard serial from bluepill pin to serial display.)
//!
//!
//! See https://github.com/eldruin/max3010x-rs for details of the driver API.
//!
//!  The data interpretation below needs some work and possibly wiring for LED power, int, etc.
//!  See  https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf
//!       https://www.instructables.com/Pulse-Oximeter-With-Much-Improved-Precision/
//!
//! Bluepill setting are 
//! ```
//! BP   <-> MAX30102 <-> Serial
//! GND  <-> GND      <-> GND
//! 3.3V <-> VCC      <-> VDD
//! PB6 (TX)          <-> RX    9600 baud  PB7 (RX) is not used
//! PB8  <-> SCL
//! PB9  <-> SDA
//!  need also LED power connections, etc.
//! ```
//!
//! Run with:
//! `cargo embed --example max30102-heart-usart-bp`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::fmt::Write as Writeln;  // for writeln!
use cortex_m_rt::entry;
use max3010x::{Led, LedPulseWidth, Max3010x, SampleAveraging, SamplingRate};
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use panic_rtt_target as _;  THIS CAUSE LINK PROBLEM ... undefined symbol: _SEGGER_RTT

//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use nb::block;

pub trait LED {
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // implementation should deal with this difference
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off();
        delay.delay_ms(time); //consider delay.delay_ms(500u16);
    }
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART1},
    prelude::*,
    serial::{Rx, Serial, Tx},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    let gpiob = dp.GPIOB.split(&mut rcc); // for i2c scl and sda

    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (
            gpiob.pb8.into_alternate_af1(cs), // scl on PB8
            gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), &mut rcc);

    let delay = Delay::new(cp.SYST, &rcc);

    // led
    let gpioc = dp.GPIOC.split(&mut rcc);
    let led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx, rx) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa9.into_alternate_af1(cs),  //tx pa9
            gpioa.pa10.into_alternate_af1(cs), //rx pa10
        )
    });

    let (tx, rx) = Serial::usart1(dp.USART1, (tx, rx), 9600.bps(), &mut rcc).split();

    (i2c, led, delay, tx, rx)
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    timer::SysDelay as Delay,
    device::USART1,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{BlockingI2c, DutyCycle, Mode, Pins},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .pclk1(36.MHz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();

    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let mut gpioc = dp.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let delay = cp.SYST.delay(&clocks);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;
    let serial = Serial::new(
        dp.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        &clocks,
    );
    let (tx, rx) = serial.split();

    (i2c, led, delay, tx, rx)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioe::PE9, Output, PushPull},
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART1},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    Delay,
    Tx<USART1, impl TxPin<USART1>>,
    Rx<USART1, impl RxPin<USART1>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let led = gpioe
        .pe9
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let delay = Delay::new(cp.SYST, clocks);

    impl LED for PE9<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let scl = gpiob
        .pb6
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    //    // not sure if pull up is needed
    //    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 400_000.Hz(), clocks, &mut rcc.apb1);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let (tx, rx) = Serial::new(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            gpioa
                .pa10
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        ),
        9600.Bd(),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    (i2c, led, delay, tx, rx)
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    timer::SysDelay as Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C2, USART1},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

//#[cfg(feature = "stm32f4xx")]
//use embedded_hal::blocking::serial::Write;
//use embedded_hal::serial::nb::Write;

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    I2c<I2C2, impl Pins<I2C2>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split(); // for i2c

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    let scl = gpiob.pb10.into_alternate().set_open_drain(); // scl on PB10
    let sda = gpiob.pb3.into_alternate().set_open_drain(); // sda on PB3

    let i2c = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

    //let delay = Delay::new(cp.SYST, &clocks);
    let delay = cp.SYST.delay(&clocks);

    // led
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

    let gpioa = dp.GPIOA.split();

    dp.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx, rx) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap()
    .split();

    (i2c, led, delay, tx, rx)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    timer::SysDelay as Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{BlockingI2c, Mode, PinScl, PinSda},
    pac::{CorePeripherals, Peripherals, I2C1, USART1},
    prelude::*,
    serial::{Config, Oversampling, Rx, Serial, Tx, DataBits, Parity},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let scl = gpiob.pb8.into_alternate_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_open_drain(); // sda on PB9

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut rcc.apb1,
        1000,
    );

    let delay = cp.SYST.delay(&clocks);

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let gpioa = dp.GPIOA.split();

    let (tx, rx) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(),
            gpioa.pa10.into_alternate(),
        ),
        &clocks,
        Config {
            baud_rate: 9600.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
    )
    .split();

    (i2c, led, delay, tx, rx)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    timer::delay::Delay,
    gpio::{Output, OpenDrain, PushPull, 
           gpiob::{PB10, PB11},
           gpioc::PC13,
    },
    i2c::{I2c, Config as i2cConfig},
    pac::{CorePeripherals, Peripherals, I2C2, USART1, TIM2},
    prelude::*,
    serial::{FullConfig, Tx, Rx},
};

#[cfg(feature = "stm32g0xx")]
fn setup() -> (
    I2c<I2C2, PB11<Output<OpenDrain>>, PB10<Output<OpenDrain>>>, 
    impl LED,
    Delay<TIM2>,
    Tx<USART1, FullConfig>,
    Rx<USART1, FullConfig>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    //  i2c
    let gpiob = dp.GPIOB.split(&mut rcc); // for i2c
    let scl = gpiob.pb10.into_open_drain_output();
    let sda = gpiob.pb11.into_open_drain_output();
    let i2c = I2c::i2c2(dp.I2C2,  sda, scl,  i2cConfig::with_timing(0x2020_151b), &mut rcc);

    let delay = dp.TIM2.delay(&mut rcc);

    // led
    let gpioc = dp.GPIOC.split(&mut rcc);
    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }

        fn off(&mut self) -> () {
            self.set_high()
        }

        fn blink(&mut self, time: u16, delay: &mut Delay<TIM2>) -> () {
            self.on();
            delay.delay_ms(time);
            self.off();
            delay.delay_ms(time); //consider delay.delay_ms(500u16);
        }
    }

    // (tx, rx) 
    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx, rx) = dp.USART1.usart((gpioa.pa9, gpioa.pa10),
                        FullConfig::default(), &mut rcc).unwrap().split();


    (i2c, led, delay, tx, rx)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    timer::delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C2, USART1},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32g4xx")]
fn setup() -> (
    I2c<I2C2, impl Pins<I2C2>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split(); // for i2c

    let scl = gpiob.pb10.into_alternate().set_open_drain(); // scl on PB10
    let sda = gpiob.pb3.into_alternate().set_open_drain(); // sda on PB3

    let i2c = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

    let delay = cp.SYST.delay(&clocks);

    // led
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

    let gpioa = dp.GPIOA.split();

    dp.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx, rx) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap()
    .split();

    (i2c, led, delay, tx, rx)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1, USART1},
    prelude::*,
    serial::{Rx, Tx},
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (I2c<I2C1>, impl LED, Delay, Tx<USART1>, Rx<USART1>) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let scl = gpiob.pb8.into_alternate().set_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate().set_open_drain(); // sda on PB9

    let i2c = dp
        .I2C1
        .i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &clocks);

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let (tx, rx) = dp
        .USART1
        .serial(
            (
                gpioa.pa9.into_alternate(),
                gpioa.pa10.into_alternate(),
            ),
            9600.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split();

    (i2c, led, delay, tx, rx)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB8, PB9},
        gpioc::PC13,
        OpenDrain, Output, PushPull,
    },
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1, USART1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, Serial1Ext, Tx},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    I2c<I2C1, PB9<Output<OpenDrain>>, PB8<Output<OpenDrain>>>,
    //I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let clocks = rcc.clocks;

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    // could also have scl on PB6, sda on PB7
    //BlockingI2c::i2c1(
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    let i2c = dp.I2C1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx, rx) = dp
        .USART1
        .usart(
            gpioa.pa9,
            gpioa.pa10,
            Config::default().baudrate(9600.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    (i2c, led, delay, tx, rx)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpiob::PB6, Output, PushPull},
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    serial::{Config, Rx, SerialExt, Tx},
    //gpio::{gpiob::{PB8, PB9}, Output, OpenDrain, },
    stm32::{CorePeripherals, Peripherals, I2C1, USART1},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    let clocks = rcc.clocks;

    let gpiob = dp.GPIOB.split(&mut rcc);

    // could also have scl,sda  on PB6,PB7 or on PB10,PB11
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let led = gpiob.pb6.into_push_pull_output(); // led on pb6

    impl LED for PB6<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }
    let gpioa = dp.GPIOA.split(&mut rcc);

    // Note that setting the alternate function mode and push_pull input/output
    // is not necessary. The hal code knows to do this for a usart.
    let (tx, rx) = dp
        .USART1
        .usart(
            (gpioa.pa9, gpioa.pa10),
            Config::default().baudrate(9600.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    (i2c, led, delay, tx, rx)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{Config as i2cConfig, I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C2, USART1},
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    I2c<I2C2, (impl SclPin<I2C2>, impl SdaPin<I2C2>)>,
    impl LED,
    Delay,
    Tx<USART1>,
    Rx<USART1>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);

    // following ttps://github.com/stm32-rs/stm32l4xx-hal/blob/master/examples/i2c_write.rs
    let mut scl =
        gpiob
            .pb10
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // scl on PB10
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let mut sda =
        gpiob
            .pb11
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // sda on PB11
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = I2c::i2c2(
        dp.I2C2,
        (scl, sda),
        i2cConfig::new(400.kHz(), clocks),
        &mut rcc.apb1r1,
    );

    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let led = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let (tx, rx) = Serial::usart1(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        ),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    (i2c, led, delay, tx, rx)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (i2c, mut led, mut delay, mut tx, _rx) = setup();

    // Blink LED to indicate running.
    led.blink(500_u16, &mut delay);

    //rtt_init_print!();
    //rprintln!("test write to console ...");
    hprintln!("test write to console ...").unwrap();
    for byte in b"\r\nconsole connect check.\r\n" {
        block!(tx.write(*byte)).ok();
    }

    let mut max30102 = Max3010x::new_max30102(i2c);
    max30102.reset().unwrap();
    delay.delay_ms(100_u8);

    let mut max30102 = max30102.into_heart_rate().unwrap();

    max30102.enable_fifo_rollover().unwrap();
    max30102.set_pulse_amplitude(Led::All, 15).unwrap();
    max30102.set_sample_averaging(SampleAveraging::Sa8).unwrap();
    max30102.set_sampling_rate(SamplingRate::Sps100).unwrap();
    max30102.set_pulse_width(LedPulseWidth::Pw411).unwrap();

    max30102.clear_fifo().unwrap();

    //writeln!(tx, "hr\r").unwrap();
    for byte in b"\r\n" {
        block!(tx.write(*byte)).unwrap();
    }

    loop {
        // Blink LED to check that loop is arunning.
        led.blink(50_u16, &mut delay);

        hprintln!("loop i").unwrap();
        //rprintln!("{}\r", data[i]);   // rprintln requires updated probe??
        delay.delay_ms(100_u8);
        let mut data = [0; 16];
        let read = max30102.read_fifo(&mut data).unwrap_or(0);
        for i in 0..read.into() {
              hprintln!("data {}", data[i]).unwrap();
              writeln!(tx, "{}\r", data[i],).unwrap();  
              //block!(tx.write(data[i] as u8)).unwrap(); //u32 to u8, may panic
        }
        for byte in b"\r\n" {
            block!(tx.write(*byte)).ok();
        }

        delay.delay_ms(3000_u16);
    }
}
