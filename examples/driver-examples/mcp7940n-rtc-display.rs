//! Stores the date and time on a MCP7940N real-time clock (RTC).
//! Then continuously print the date and time.
//!
//! Introductory blog post here:
//! https://blog.eldruin.com/mcp794xx-real-time-clock-rtc-driver-in-rust/
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP    <-> MCP7940N <-> Display
//! GND   <-> GND      <-> GND
//! +3.3V <-> +3.3V    <-> +3.3V
//! PB8   <-> SCL      <-> SCL
//! PB9   <-> SDA      <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use mcp794xx::{Datelike, Mcp794xx, NaiveDate, Rtcc, Timelike};

use core::fmt::Write;
use cortex_m_rt::entry;

use rtt_target::{rprintln, rtt_init_print};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("MCP7940N example");

    let (i2c, mut led, mut delay) = setup();

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut rtc = Mcp794xx::new_mcp7940n(manager.acquire_i2c());
    let begin = NaiveDate::from_ymd(2019, 1, 2).and_hms(4, 5, 6);
    rtc.set_datetime(&begin).unwrap();
    rtc.enable().unwrap();
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        let now = rtc.get_datetime().unwrap();

        let mut buffer: heapless::String<32> = heapless::String::new();
        write!(
            buffer,
            "{}-{}-{} {}:{}:{}   ",
            now.year(),
            now.month(),
            now.day(),
            now.hour(),
            now.minute(),
            now.second()
        )
        .unwrap();
        display.clear();
        Text::new(&buffer, Point::zero(), text_style)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
    }
}
