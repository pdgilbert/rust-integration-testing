//! Based on example  https://github.com/eldruin/driver-examples/stm32f3-discovery/examples/mcp42x-f3.rs
//!  which runs on the STM32F3 Discovery board using SPI1.
//!
//! ```
//! F3   <-> MCP42x
//! GND  <-> VSS
//! 3.3V <-> VDD
//! PA5  <-> CLK
//! PA7  <-> SI
//! PB5  <-> CS
//! ```
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

// use nb::block;
use cortex_m_rt::entry;
use cortex_m_semihosting::*;
//use asm_delay::{ Delay, bitrate, };
//use cortex_m::asm;  //for breakpoint

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;

//use f3::{
//    hal::{
//        delay::Delay, flash::FlashExt, gpio::GpioExt, rcc::RccExt, spi::Spi, stm32f30x,
//        time::U32Ext,
//    },
//    led::Led,
//};

use mcp4x;

use mcp4x::{Channel, Mcp4x, MODE};

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{
        gpioa::{PA0, PA1},
        gpioa::{PA5, PA6, PA7},
        gpioe::PE9,
        Output, PushPull, AF5,
    },
    pac::SPI1,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    spi::Spi,
};

#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    //let p = stm32f30x::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);

    // SPI configuration
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let spi = Spi::spi1(
        p.SPI1,
        (sck, miso, mosi),
        MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut chip_select = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    chip_select.set_high().unwrap();

    let mut pot = Mcp4x::new_mcp42x(spi, chip_select);

    let mut delay = Delay::new(cp.SYST, clocks);

    let mut position = 0;
    loop {
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
