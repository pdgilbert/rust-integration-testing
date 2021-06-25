//! Continuously measure the ultraviolet A and ultraviolet B light sensor data
//! and print it to an SSD1306 OLED display together with the calculated
//! UV index.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/veml6075-uva-uvb-uv-index-light-sensor-driver-in-rust/
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs. 
//!  The specific function used will depend on the HAL setting (see README.md). 
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> VEML6075 <-> Display
//! GND  <-> GND      <-> GND
//! 3.3V <-> VCC      <-> VDD
//! PB8  <-> SCL      <-> SCL
//! PB9  <-> SDA      <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use veml6075::{Calibration, Measurement, Veml6075};

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use hal_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("VEML6075 example");

    let (i2c, mut led, mut delay) = setup();

    let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut sensor = Veml6075::new(manager.acquire(), Calibration::default());

    let mut lines: [heapless::String<32>; 3] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];
    sensor.enable().unwrap();
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        // If there was an error, it will print 0.00, 0.00, 0.00.
        let Measurement { uva, uvb, uv_index } = sensor.read().unwrap_or(Measurement {
            uva: 0.0,
            uvb: 0.0,
            uv_index: 0.0,
        });

        lines[0].clear();
        lines[1].clear();
        lines[2].clear();

        write!(lines[0], "UVA: {}", uva).unwrap();
        write!(lines[1], "UVB: {}", uvb).unwrap();
        write!(lines[2], "UV index: {}", uv_index).unwrap();
        display.clear();
        for (i, line) in lines.iter().enumerate() {
            Text::new(line, Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
