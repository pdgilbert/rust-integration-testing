//! Continuously measure the temperature and humidity with an
//! HDC2080 sensor and print the values to an SSD1306 OLED display.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> HDC2080 <-> Display
//! GND  <-> GND     <-> GND
//! 3.3V <-> VCC     <-> VDD
//! PB8  <-> SCL     <-> SCL
//! PB9  <-> SDA     <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use hdc20xx::{Hdc20xx, SlaveAddr};

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use cortex_m_rt::entry;
use nb::block;

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
    rprintln!("HDC2080 example");

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

    let mut sensor = Hdc20xx::new(manager.acquire_i2c(), SlaveAddr::default());
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);
        delay.delay_ms(50_u16);

        let data = block!(sensor.read()).unwrap();

        lines[0].clear();
        lines[1].clear();
        write!(lines[0], "Temperature: {:.2}ºC  ", data.temperature).unwrap();
        write!(lines[1], "Humidity: {:.2}%  ", data.humidity.unwrap()).unwrap();
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::new(line, Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
