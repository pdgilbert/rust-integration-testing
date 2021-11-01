//! Using ads1x1x to read voltages measured by ads1015/1115 ADC on i2c
//! and  crate ssd1306 to print with i2c on a generic ssd1306 based OLED display.
//!
//! Uses the `embedded_graphics` crate to draw.
//! Wiring pin connections for scl and sda to display as in the setup sections below.
//! The full DisplaySize 128x64 is used. Writing will need to be adjusted if display size is smaller.
//! This example took guidance from
//!    https://github.com/jamwaffles/ssd1306/blob/master/examples/text_i2c.rs  and
//!    https://github.com/eldruin/driver-examples/stm32f1-bluepill/examples/ads1015-adc-display-bp.rs
//!  See https://blog.eldruin.com/ads1x1x-analog-to-digital-converter-driver-in-rust/
//!    for much more detailed description.

// Example use of impl trait: If scl and sda are on PB8 and PB9 (eg in stm32f1xx below) then
//    fn setup() ->  (BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>,
//    PC13<Output<PushPull>>, Delay ) {
// is changed to
//    fn setup() ->  BlockingI2c<I2C1, impl Pins<I2C1>>, PC13<Output<PushPull>>, Delay ) {
// Also
//   use stm32f1xx_hal::{ gpio::{gpiob::{PB8, PB9}, Alternate, OpenDrain, },
// will be needed.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use ads1x1x::{Ads1x1x, ChannelSelection, DynamicOneShot, FullScaleRange, SlaveAddr};

use core::fmt::Write;

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle, MonoTextStyleBuilder}, //FONT_6X10  FONT_8X13
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use nb::block;

pub trait LED {
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off()
    }
    fn blink_ok(&mut self, delay: &mut Delay) -> () {
        let dot: u16 = 5;
        let dash: u16 = 200;
        let spc: u16 = 500;
        let space: u16 = 1000;
        let end: u16 = 1500;

        // dash-dash-dash
        self.blink(dash, delay);
        delay.delay_ms(spc);
        self.blink(dash, delay);
        delay.delay_ms(spc);
        self.blink(dash, delay);
        delay.delay_ms(space);

        // dash-dot-dash
        self.blink(dash, delay);
        delay.delay_ms(spc);
        self.blink(dot, delay);
        delay.delay_ms(spc);
        self.blink(dash, delay);
        delay.delay_ms(end);
    }
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();

    let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);

    let gpiob = p.GPIOB.split(&mut rcc); // for i2c scl and sda

    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (
            gpiob.pb8.into_alternate_af1(cs), // scl on PB8
            gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });

    let i2c = I2c::i2c1(p.I2C1, (scl, sda), 400.khz(), &mut rcc);

    let delay = Delay::new(cp.SYST, &rcc);

    // led
    let gpioc = p.GPIOC.split(&mut rcc);
    let led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    delay::Delay,
    device::I2C2,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{BlockingI2c, DutyCycle, Mode, Pins},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (BlockingI2c<I2C2, impl Pins<I2C2>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);

    let mut gpiob = p.GPIOB.split(); // for i2c scl and sda

    // can have (scl, sda) using I2C1  on (PB8, PB9 ) or on  (PB6, PB7)
    //     or   (scl, sda) using I2C2  on (PB10, PB11)

    let i2c = BlockingI2c::i2c2(
        p.I2C2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 but not i2c2
        Mode::Fast {
            frequency: 100_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let mut gpioc = p.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioe::PE15, Output, PushPull},
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb); // for i2c

    let mut scl =
        gpiob
            .pb8
            .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // scl on PB8
    let mut sda =
        gpiob
            .pb9
            .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // sda on PB9

    // not sure if pull up is needed
    scl.internal_pull_up(&mut gpiob.pupdr, true);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = I2c::new(
        p.I2C1,
        (scl, sda),
        //&mut afio.mapr,  need this for i2c1 but not i2c2
        400_000.Hz(),
        //100u32.kHz().try_into().unwrap(),
        clocks,
        &mut rcc.apb1,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb);
    let led = gpioe
        .pe15
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper); // led on pe15

    impl LED for PE15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C2},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (I2c<I2C2, impl Pins<I2C2>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = p.GPIOB.split(); // for i2c

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    let scl = gpiob.pb10.into_alternate().set_open_drain(); // scl on PB10
    let sda = gpiob.pb3.into_alternate().set_open_drain(); // sda on PB3

    let i2c = I2c::new(p.I2C2, (scl, sda), 400.khz(), clocks);

    let delay = Delay::new(cp.SYST, &clocks);

    // led
    let gpioc = p.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{BlockingI2c, Mode, PinScl, PinSda},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();

    let scl = gpiob.pb8.into_alternate_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_open_drain(); // sda on PB9

    let i2c = BlockingI2c::i2c1(
        p.I2C1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        clocks,
        &mut rcc.apb1,
        1000,
    );

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

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (I2c<I2C1>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;

    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = p.GPIOC.split(ccdr.peripheral.GPIOC);

    let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain(); // sda on PB9

    let i2c = p
        .I2C1
        .i2c((scl, sda), 400.khz(), ccdr.peripheral.I2C1, &clocks);

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, SCLPin, SDAPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let clocks = rcc.clocks;

    let gpiob = p.GPIOB.split(&mut rcc);
    let gpioc = p.GPIOC.split(&mut rcc);

    // could also have scl on PB6, sda on PB7
    //BlockingI2c::i2c1(
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    let i2c = p.I2C1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

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

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpiob::PB6, Output, PushPull},
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    stm32::{CorePeripherals, Peripherals, I2C1},
    //gpio::{gpiob::{PB8, PB9}, Output, OpenDrain, },
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
fn setup() -> (I2c<I2C1, impl Pins<I2C1>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());
    let clocks = rcc.clocks;

    let gpiob = p.GPIOB.split(&mut rcc);

    // could also have scl,sda  on PB6,PB7 or on PB10,PB11
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    let i2c = p.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

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

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{
        gpioc::PC13,
        //gpiob::{PB10, PB11},
        Output,
        PushPull, //Alternate, OpenDrain, AF4,
    },
    i2c::{Config as i2cConfig, I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C2},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    I2c<I2C2, (impl SclPin<I2C2>, impl SdaPin<I2C2>)>,
    impl LED,
    Delay,
) {
    //fn setup() ->  (I2c<I2C2, (PB10<Alternate<AF4, OpenDrain>>, PB11<Alternate<AF4, OpenDrain>>)>,
    //    PC13<Output<PushPull>>, Delay ) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // following ttps://github.com/stm32-rs/stm32l4xx-hal/blob/master/examples/i2c_write.rs
    let mut scl =
        gpiob
            .pb10
            .into_af4_opendrain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // scl on PB10
    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //let scl = scl.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let mut sda =
        gpiob
            .pb11
            .into_af4_opendrain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // sda on PB11
    sda.internal_pull_up(&mut gpiob.pupdr, true);
    //let sda = sda.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let i2c = I2c::i2c2(
        p.I2C2,
        (scl, sda),
        i2cConfig::new(400.khz(), clocks),
        &mut rcc.apb1r1,
    );

    let mut gpioc = p.GPIOC.split(&mut rcc.ahb2);

    let delay = Delay::new(cp.SYST, clocks);

    // led
    let led = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

// End of hal/MCU specific setup. Following should be generic code.

pub fn read_all<E, A: DynamicOneShot<Error = E>>(
    adc_a: &mut A,
    adc_b: &mut A,
) -> (i16, i16, i16, [i16; 3]) {
    // Note scale_cur divides, scale_a and scale_b multiplies
    let scale_cur = 10; // calibrated to get mA/mV depends on FullScaleRange above and values of shunt resistors
                        //let scale_a = 2; // calibrated to get mV    depends on FullScaleRange
    let scale_b = 2; // calibrated to get mV    depends on FullScaleRange

    //TMP35 scale is 100 deg C per 1.0v (slope 10mV/deg C) and goes through
    //     <50C, 1.0v>,  so 0.0v is  -50C.

    let scale_temp = 5; //divides
    let offset_temp = 50;

    //first adc  Note that readings are zero on USB power (programming) rather than battery.

    let bat_ma = block!(adc_a.read(ChannelSelection::DifferentialA1A3)).unwrap_or(8091) / scale_cur;
    let load_ma =
        block!(adc_a.read(ChannelSelection::DifferentialA2A3)).unwrap_or(8091) / scale_cur;

    // toggle FullScaleRange to measure battery voltage, not just diff across shunt resistor
    // also first adc
    // skip until working see
    //    https://github.com/eldruin/ads1x1x-rs/issues/10?_pjax=%23repo-content-pjax-container
    //adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    //let bat_mv = block!(adc_a.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_a;
    //adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

    // second adc
    let values_b = [
        block!(adc_b.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_b,
        block!(adc_b.read(ChannelSelection::SingleA1)).unwrap_or(8091) * scale_b,
        block!(adc_b.read(ChannelSelection::SingleA2)).unwrap_or(8091) * scale_b,
    ];

    let temp_c =
        block!(adc_b.read(ChannelSelection::SingleA3)).unwrap_or(8091) / scale_temp - offset_temp;

    // third adc
    //let values_c = [
    //    block!(adc_c.read(ChannelSelection::SingleA0)).unwrap_or(8091),
    //    block!(adc_c.read(ChannelSelection::SingleA1)).unwrap_or(8091),
    //    block!(adc_c.read(ChannelSelection::SingleA2)).unwrap_or(8091),
    //    block!(adc_c.read(ChannelSelection::SingleA3)).unwrap_or(8091),
    //];

    //(bat_mv, bat_ma, load_ma, temp_c, values_b )
    (bat_ma, load_ma, temp_c, values_b)
}

fn display<S>(
    bat_mv: i16,
    bat_ma: i16,
    load_ma: i16,
    temp_c: i16,
    values_b: [i16; 3],
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 4] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // it is now possible to use \n in place of separate writes, with one line rather than vector.
    write!(lines[0], "bat:{:4}mV{:4}mA", bat_mv, bat_ma).unwrap();
    write!(lines[1], "load:    {:5}mA", load_ma).unwrap();
    write!(
        lines[2],
        "B:{:4} {:4} {:4}",
        values_b[0], values_b[1], values_b[2]
    )
    .unwrap();
    write!(lines[3], "temperature{:3} C", temp_c).unwrap();

    disp.clear();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 16),
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}

#[entry]
fn main() -> ! {
    let (i2c, mut led, mut delay) = setup();

    led.blink_ok(&mut delay); // blink OK to indicate setup complete and main started.

    let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire());

    let mut disp = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    disp.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13) //.&FONT_6X10  &FONT_8X13
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        "Display initialized ...",
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut disp)
    .unwrap();

    let mut adc_a = Ads1x1x::new_ads1015(manager.acquire(), SlaveAddr::Alternative(false, false)); //addr = GND
    let mut adc_b = Ads1x1x::new_ads1015(manager.acquire(), SlaveAddr::Alternative(false, true)); //addr =  V

    // set FullScaleRange to measure expected max voltage.
    // This is very small for diff across low value shunt resistors
    //   but up to 5v for single pin with usb power.
    // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v
    adc_a
        .set_full_scale_range(FullScaleRange::Within0_256V)
        .unwrap();
    //adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    adc_b
        .set_full_scale_range(FullScaleRange::Within4_096V)
        .unwrap();
    //adc_c.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

    let scale_a = 2; // calibrated to get mV    depends on FullScaleRange

    loop {
        // Blink LED to check that everything is actually running.
        // Note that blink takes about a mA current and makes measurement below noisy.
        // Comment out blinking to calibrate scale.
        led.blink(10_u16, &mut delay);

        adc_a
            .set_full_scale_range(FullScaleRange::Within4_096V)
            .unwrap();
        //let bat_mv = block!(adc_a.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_a;
        let bat_mv = block!(DynamicOneShot::read(&mut adc_a, ChannelSelection::SingleA0))
            .unwrap_or(8091)
            * scale_a;
        adc_a
            .set_full_scale_range(FullScaleRange::Within0_256V)
            .unwrap();

        let (bat_ma, load_ma, temp_c, values_b) = read_all(&mut adc_a, &mut adc_b);

        display(
            bat_mv, bat_ma, load_ma, temp_c, values_b, text_style, &mut disp,
        );

        delay.delay_ms(2000_u16); // sleep for 2s
    }
}
