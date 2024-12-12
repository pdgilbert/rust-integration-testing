//! This is to test impl trait for returning spi and cs. It does not run, only builds.
//!
//!  Also it fails linking, I think because there is no target on the spi bus.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use panic_rtt_target as _;  THIS CAUSE LINK PROBLEM ... undefined symbol: _SEGGER_RTT

//use embedded_hal::spi::{Mode, Phase, Polarity}; THIS CONFLICTS WITH VERSION FROM HAL
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    gpio::{
        gpioa::{PA1, PA5, PA6, PA7},
        Alternate, Output, PushPull, AF0,
    },
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{EightBit, MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    Spi<SPI1, PA5<Alternate<AF0>>, PA6<Alternate<AF0>>, PA7<Alternate<AF0>>, EightBit>,
    PA1<Output<PushPull>>,
) {
    //fn setup() -> (Spi<SPI1, impl SckPin<SPI1>, MisoPin<SPI1>, MosiPin<SPI1>, EightBit>, PA1<Output<PushPull>>) {
    //fn setup() -> (Spi<SPI1, impl Pins<SPI1>, MisoPin<SPI1>, MosiPin<SPI1>, EightBit>, PA1<Output<PushPull>> ) {
    //fn setup() -> (Spi<SPI1, impl Pins<SPI1>, EightBit>, PA1<Output<PushPull>> ) {
    let mut dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (sck, miso, mosi, chs) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa5.into_alternate_af0(cs),    //   sck   on PA5
            gpioa.pa6.into_alternate_af0(cs),    //   miso  on PA6
            gpioa.pa7.into_alternate_af0(cs),    //   mosi  on PA7
            gpioa.pa1.into_push_pull_output(cs), // cs   on PA1
        )
    });

    let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), MODE, 8.mhz(), &mut rcc);

    (spi, chs)
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    gpio::gpioa::PA4,
    gpio::{Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{Spi, Mode, Phase, Polarity},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Spi<SPI1, u8>,
    //Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>), u8>,
    //Spi<SPI1, Spi1NoRemap, impl Pins<Spi1NoRemap>, u8>,
    PA4<Output<PushPull>>,
) {
    //fn setup() -> (Spi<SPI1, Spi1NoRemap, (PA5<Alternate<PushPull>>, PA6<Input<Floating>>, PA7<Alternate<PushPull>>), u8>,
    //    PA4<Output<PushPull>>) {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
//let spi = dp
//        .SPI1
//        //.remap(&mut afio.mapr) // if you want to use PB3, PB4, PB5
//        .spi((Some(sck), Some(miso), Some(mosi)), MODE, 1.MHz(), &clocks);

    let spi =  dp
                .SPI1
                //.remap(&mut afio.mapr) // if you want to use PB3, PB4, PB5
                .spi((Some(sck), Some(miso), Some(mosi)),  MODE, 1.MHz(), &clocks );

    (spi, cs)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>), u8>,
    PA1<Output<PushPull>>,
) {
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut dp.FLASH.constrain().acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa
                .pa5
                .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
            gpioa
                .pa6
                .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // miso  on PA6
            gpioa
                .pa7
                .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // mosi  on PA7
        ),
        MODE,
        8_000_000.Hz(),
        clocks,
        &mut rcc.apb2,
    );

    let cs = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    (spi, cs)
}

#[cfg(feature = "stm32f4xx")]
// eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi},
    time::MegaHertz,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (Spi<SPI1, impl Pins<SPI1>>, PA1<Output<PushPull>>) {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();

    let gpioa = dp.GPIOA.split();

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa.pa5.into_alternate_af5(), // sck   on PA5
            gpioa.pa6.into_alternate_af5(), // miso  on PA6
            gpioa.pa7.into_alternate_af5(), // mosi  on PA7
        ),
        MODE,
        MegaHertz(8).into(),
        clocks,
    );

    let cs = gpioa.pa1.into_push_pull_output();

    (spi, cs)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{ClockDivider, Enabled, Pins, Spi},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>, Enabled<u8>>,
    PA1<Output<PushPull>>,
) {
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split();

    let sck = gpioa.pa5.into_alternate_af5(); // sck   on PA5
    let miso = gpioa.pa6.into_alternate_af5(); // miso  on PA6
    let mosi = gpioa.pa7.into_alternate_af5(); // mosi  on PA7

    //   somewhere 8.mhz needs to be set in spi

    let spi =
        Spi::new(dp.SPI1, (sck, miso, mosi)).enable::<u8>(&mut rcc.apb2, ClockDivider::DIV32, MODE);

    let cs = gpioa.pa1.into_push_pull_output();

    (spi, cs)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi},
};

#[cfg(feature = "stm32g0xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>>,
    PA1<Output<PushPull>>,
) {
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let spi = dp.SPI1.spi((gpioa.pa5, gpioa.pa6, gpioa.pa7), //sck, miso, mosi
        mcp4x::MODE, 
        3.MHz(), &mut rcc, );

    let mut cs = gpioa.pa1.into_push_pull_output();
    cs.set_high().unwrap();

    (spi, cs)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    gpio::{gpioa::PA8, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi, MODE_0},
    time::{RateExtU32},
};

#[cfg(feature = "stm32g4xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>>,
    PA8<Output<PushPull>>,
) {
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let spi = dp.SPI1.spi(
            (gpioa.pa5.into_alternate(), // sck   on PA5
             gpioa.pa6.into_alternate(), // miso  on PA6
             gpioa.pa7.into_alternate(), // mosi  on PA7
            ),
        MODE_0,
        400.kHz(),
        &mut rcc,
    );

    let mut cs = gpioa.pa8.into_push_pull_output();
    cs.set_high().unwrap();

    (spi, cs)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{Enabled, Spi},
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (Spi<SPI1, Enabled>, PA1<Output<PushPull>>) {
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5.into_alternate_af5(), // sck   on PA5
            gpioa.pa6.into_alternate_af5(), // miso  on PA6
            gpioa.pa7.into_alternate_af5(), // mosi  on PA7
        ),
        MODE,
        8.mhz(),
        ccdr.peripheral.SPI1,
        &clocks,
    );

    let cs = gpioa.pa1.into_push_pull_output();

    (spi, cs)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    spi::{Pins, Spi},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (Spi<SPI1, impl Pins<SPI1>>, PA1<Output<PushPull>>) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5
            gpioa.pa6, // miso  on PA6
            gpioa.pa7, // mosi  on PA7
        ),
        MODE,
        8.mhz(),
        &mut rcc,
    );

    let cs = gpioa.pa1.into_push_pull_output();

    (spi, cs)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    gpio::{gpioa::PA4, Output, PushPull},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    spi::{Pins, Spi},
    stm32::{Peripherals, SPI1},
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (Spi<SPI1, impl Pins<SPI1>>, PA4<Output<PushPull>>) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

    let gpioa = dp.GPIOA.split();

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5   in board on Heltec
            gpioa.pa6, // miso  on PA6   in board on Heltec
            gpioa.pa7, // mosi  on PA7   in board on Heltec
        ),
        MODE,
        8.mhz(),
        &mut rcc,
    );

    let cs = gpioa.pa4.into_push_pull_output();

    (spi, cs)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    gpio::{gpioa::PA1, Output, PushPull},
    pac::{Peripherals, SPI1},
    prelude::*,
    spi::{MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>)>,
    PA1<Output<PushPull>>,
) {
    let dp = Peripherals::take().unwrap();
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

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa
                .pa5
                .into_af5_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
            gpioa
                .pa6
                .into_af5_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // miso  on PA6
            gpioa
                .pa7
                .into_af5_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // mosi  on PA7
        ),
        MODE,
        8.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let cs = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    (spi, cs)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (_spi, _cs) = setup();

    loop {}
}
