//! Based on example  https://github.com/eldruin/driver-examples/stm32f3-discovery/examples/mcp42x-f3.rs
//!  which runs on the STM32F3 Discovery board using SPI1. On that board the harware configuration is
//!
//! ```
//! F3   <-> MCP42x
//! GND  <-> VSS
//! 3.3V <-> VDD
//! PA5  <-> CLK
//! PA7  <-> SI
//! PB5  <-> CS
//! ```
//! See setup sections below for other boards.
//!
//! Loop setting a position from 0 to 255 to the channel 0 of a MCP42010
//! digital potentiometer and its inverse to channel 1.
//!
//! To use the device as a variable resistor (rheostat configuration) connect
//! PW0 to PA0 and measure the resistence between PA0 and PB0.
//! You should see that the resistence varies from 0 to 10K ohm for an MCP42010.
//! The maximum value will be different depending on the exact model.
//! For example, 0-50Kohm for MCP42050 and 0-100Kohm for MCP42100.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
//use cortex_m_semihosting::*;

use embedded_hal::blocking::delay::DelayMs;

use mcp4x;

pub trait LED {
    // depending on board wiring, `on` may be `set_high` or `set_low`, with `off` also reversed
    // implementation should deal with this difference
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off()
    }
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpioa::{PA1, PA5, PA6, PA7},
        gpioc::PC13,
        Alternate, Output, PushPull, AF0,
    },
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{EightBit, Spi},
    //spi::{EightBit, MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    Spi<SPI1, PA5<Alternate<AF0>>, PA6<Alternate<AF0>>, PA7<Alternate<AF0>>, EightBit>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    //fn setup() -> (Spi<SPI1, impl SckPin<SPI1>, MisoPin<SPI1>, MosiPin<SPI1>, EightBit>, PA1<Output<PushPull>>) {
    //fn setup() -> (Spi<SPI1, impl Pins<SPI1>, MisoPin<SPI1>, MosiPin<SPI1>, EightBit>, PA1<Output<PushPull>> ) {
    //fn setup() -> (Spi<SPI1, impl Pins<SPI1>, EightBit>, PA1<Output<PushPull>> ) {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().sysclk(8.mhz()).freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let (sck, miso, mosi, mut cs, led) = cortex_m::interrupt::free(move |c| {
        (
            gpioa.pa5.into_alternate_af0(c),     //   sck   on PA5
            gpioa.pa6.into_alternate_af0(c),     //   miso  on PA6
            gpioa.pa7.into_alternate_af0(c),     //   mosi  on PA7
            gpioa.pa1.into_push_pull_output(c),  // cs   on PA1
            gpioc.pc13.into_push_pull_output(c), // led
        )
    });

    let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), mcp4x::MODE, 8.mhz(), &mut rcc);

    cs.set_high().unwrap();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    (spi, cs, led, Delay::new(cp.SYST, &rcc))
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA4, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi, Spi1NoRemap},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Spi<SPI1, Spi1NoRemap, impl Pins<Spi1NoRemap>, u8>,
    PA4<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let mut cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    cs.set_high();

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        mcp4x::MODE,
        1_u32.MHz(),
        clocks,
    );

    let mut gpioc = dp.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let delay = Delay::new(cp.SYST, &clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpiob::PB5, gpioe::PE15, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>), u8>,
    PB5<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut dp.FLASH.constrain().acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let spi = Spi::new(
        dp.SPI1,
        (
            gpioa
                .pa5
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
            gpioa
                .pa6
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // miso  on PA6
            gpioa
                .pa7
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // mosi  on PA7
        ),
        1_000_000.Hz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut cs = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    cs.set_high().unwrap();

    let led = gpioe
        .pe15
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    impl LED for PE15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let delay = Delay::new(cp.SYST, clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32f4xx")]
// eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{
    timer::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi, TransferModeNormal},
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>, TransferModeNormal>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let spi = Spi::new(
        dp.SPI1,
        (
            gpioa.pa5.into_alternate(), // sck   on PA5
            gpioa.pa6.into_alternate(), // miso  on PA6
            gpioa.pa7.into_alternate(), // mosi  on PA7
        ),
        mcp4x::MODE,
        8.MHz(),
        &clocks,
    );

    let mut cs = gpioa.pa1.into_push_pull_output();
    cs.set_high();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    //let delay = Delay::new(cp.SYST, &clocks);
    let delay = cp.SYST.delay(&clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{ClockDivider, Enabled, Pins, Spi},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>, Enabled<u8>>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let sck = gpioa.pa5.into_alternate(); // sck   on PA5
    let miso = gpioa.pa6.into_alternate(); // miso  on PA6
    let mosi = gpioa.pa7.into_alternate(); // mosi  on PA7

    //   somewhere 8.mhz needs to be set in spi

    let spi = Spi::new(dp.SPI1, (sck, miso, mosi)).enable::<u8>(
        &mut rcc.apb2,
        ClockDivider::DIV32,
        mcp4x::MODE,
    );

    let mut cs = gpioa.pa1.into_push_pull_output();
    cs.set_high();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    (spi, cs, led, Delay::new(cp.SYST, clocks))
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{Enabled, Spi},
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (Spi<SPI1, Enabled>, PA1<Output<PushPull>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5.into_alternate_af5(), // sck   on PA5
            gpioa.pa6.into_alternate_af5(), // miso  on PA6
            gpioa.pa7.into_alternate_af5(), // mosi  on PA7
        ),
        mcp4x::MODE,
        8.mhz(),
        ccdr.peripheral.SPI1,
        &clocks,
    );

    let mut cs = gpioa.pa1.into_push_pull_output();
    cs.set_high().unwrap();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    (spi, cs, led, Delay::new(cp.SYST, ccdr.clocks))
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    spi::{Pins, Spi},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5
            gpioa.pa6, // miso  on PA6
            gpioa.pa7, // mosi  on PA7
        ),
        mcp4x::MODE,
        8_000_000.Hz(),
        &mut rcc,
    );

    let mut cs = gpioa.pa1.into_push_pull_output();
    cs.set_high().unwrap();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    (spi, cs, led, Delay::new(cp.SYST, rcc.clocks))
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA4, gpiob::PB6, Output, PushPull},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    spi::{Pins, Spi},
    stm32::{CorePeripherals, Peripherals, SPI1},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>>,
    PA4<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5   in board on Heltec
            gpioa.pa6, // miso  on PA6   in board on Heltec
            gpioa.pa7, // mosi  on PA7   in board on Heltec
        ),
        mcp4x::MODE,
        8.mhz(),
        &mut rcc,
    );

    let mut cs = gpioa.pa4.into_push_pull_output();
    cs.set_high().unwrap();

    let led = gpiob.pb6.into_push_pull_output();

    impl LED for PB6<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    (spi, cs, led, cp.SYST.delay(rcc.clocks))
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>)>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
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
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa
                .pa5
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
            gpioa
                .pa6
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // miso  on PA6
            gpioa
                .pa7
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // mosi  on PA7
        ),
        mcp4x::MODE,
        8.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut cs = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    cs.set_high();

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

    (spi, cs, led, Delay::new(cp.SYST, clocks))
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (spi, chip_select, mut led, mut delay) = setup();

    //  WARNING CS MAY NEED ON/OFF LIKE LED, RATHER THAN HIGH/LOW

    let mut pot = mcp4x::Mcp4x::new_mcp42x(spi, chip_select);

    let mut position = 0;
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 does not blink, something went wrong.
        led.blink(50_u16, &mut delay);

        delay.delay_ms(50_u16);

        pot.set_position(mcp4x::Channel::Ch0, position).unwrap();
        pot.set_position(mcp4x::Channel::Ch1, 255 - position)
            .unwrap();

        if position == 255 {
            position = 0
        } else {
            position += 1;
        }
    }
}
