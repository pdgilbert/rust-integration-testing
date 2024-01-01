//! Continuously read the object temperature with the TMP006 and display it in
//! an SSD1306 OLED display.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/tmp006-contact-less-infrared-ir-thermopile-driver-in-rust/
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> TMP006 <-> Display
//! GND  <-> GND    <-> GND
//! 3.3V <-> VCC    <-> VDD
//! PB8  <-> SCL    <-> SCL
//! PB9  <-> SDA    <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use tmp006::{SlaveAddr, Tmp006};

use core::fmt::Write;
use cortex_m_rt::entry;
use nb::block;

use rtt_target::rtt_init_print;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::pac::{Peripherals};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let dp = Peripherals::take().unwrap();

    let (i2c, _i2c2, mut led, mut delay, _clock) = i2c1_i2c2_led_delay::setup(dp);

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

    let mut tmp006 = Tmp006::new(manager.acquire_i2c(), SlaveAddr::default());

    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        lines[0].clear();
        lines[1].clear();

        let calibration_factor = 6e-14;
        let temp_k = block!(tmp006.read_object_temperature(calibration_factor)).unwrap();
        let temp_c = temp_k - 273.15;
        write!(lines[0], "Temperature: {:.2}ºC", temp_c).unwrap();

        // Read data in raw format
        let raw_data = block!(tmp006.read_sensor_data()).unwrap();
        write!(
            lines[1],
            "OV: {}, AT: {}",
            raw_data.object_voltage, raw_data.ambient_temperature
        )
        .unwrap();

        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::new(line, Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
