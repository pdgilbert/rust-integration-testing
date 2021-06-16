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
use embedded_hal::digital::v2::OutputPin;

use mcp4x;

use mcp4x::{Channel, Mcp4x, MODE};

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
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let mut cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        MODE,
        1_u32.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    cs.set_high().unwrap();

    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let delay = Delay::new(cp.SYST, clocks);

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
        1_000_000.Hz(),
        clocks,
        &mut rcc.apb2,
    );

    let cs = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

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

#[entry]
fn main() -> ! {
    let (spi, chip_select, mut led, mut delay) = setup();


//  WARNING CS MAY NEED ON/OFF LIKE LED, RATHER THAN HIGH/LOW

    chip_select.set_high().unwrap();

    let mut pot = Mcp4x::new_mcp42x(spi, chip_select);

    let mut position = 0;
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 does not blink, something went wrong.
        led.blink(50_u16, &mut delay);

        delay.delay_ms(50_u16);

        pot.set_position(Channel::Ch0, position).unwrap();
        pot.set_position(Channel::Ch1, 255 - position).unwrap();

        if position == 255 {
            position = 0
        } else {
            position += 1;
        }
    }
}
