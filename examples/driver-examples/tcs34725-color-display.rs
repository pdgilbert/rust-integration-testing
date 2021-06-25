//! Continuously read the color light sensor data and print it to
//! an SSD1306 OLED display.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs. 
//!  The specific function used will depend on the HAL setting (see README.md). 
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> TCS34725 <-> Display
//! GND  <-> GND      <-> GND
//! 3.3V <-> VCC      <-> VDD
//! PB8  <-> SCL      <-> SCL
//! PB9  <-> SDA      <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use tcs3472::{AllChannelMeasurement, Tcs3472};

use core::fmt::Write;
use cortex_m_rt::entry;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;

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
    rprintln!("TCS34725 example");

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

    let mut sensor = Tcs3472::new(manager.acquire());
    sensor.enable().unwrap();
    sensor.enable_rgbc().unwrap();
    while !sensor.is_rgbc_status_valid().unwrap() {
        // wait for measurement to be available
        delay.delay_ms(50_u8);
    }
    let mut lines: [heapless::String<32>; 4] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        // If there was an error, it will print 0, 0, 0, 0
        let measurement = sensor.read_all_channels().unwrap_or(AllChannelMeasurement {
            red: 0,
            green: 0,
            blue: 0,
            clear: 0,
        });

        lines[0].clear();
        lines[1].clear();
        lines[2].clear();
        lines[3].clear();

        write!(lines[0], "Red: {}", measurement.red).unwrap();
        write!(lines[1], "Green: {}", measurement.green).unwrap();
        write!(lines[2], "Blue: {}", measurement.blue).unwrap();
        write!(lines[3], "Clear: {}", measurement.clear).unwrap();
        display.clear();
        for (i, line) in lines.iter().enumerate() {
            Text::new(line, Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
