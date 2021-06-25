//! Continuously measure the acceleration data with an MMA8452Q
//! and print it to an SSD1306 OLED display.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs. 
//!  The specific function used will depend on the HAL setting (see README.md). 
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> MMA8452 <-> Display
//! GND  <-> GND     <-> GND
//! 3.3V <-> VCC     <-> VDD
//! PB8  <-> SCL     <-> SCL
//! PB9  <-> SDA     <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]
use mma8x5x::{Measurement, Mma8x5x, SlaveAddr};

use core::fmt::Write;
use cortex_m_rt::entry;

use rtt_target::{rprintln, rtt_init_print};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

use hal_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("MMA8452 example");

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

    let mut buffer: heapless::String<64> = heapless::String::new();
    let sensor = Mma8x5x::new_mma8452(manager.acquire(), SlaveAddr::default());
    let mut sensor = sensor.into_active().ok().unwrap();

    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        let def = Measurement {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let m = sensor.read().unwrap_or(def);

        buffer.clear();
        write!(buffer, "{:.2}, {:.2}, {:.2}", m.x, m.y, m.z).unwrap();
        display.clear();
        Text::new(&buffer, Point::zero(), text_style)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
    }
}
