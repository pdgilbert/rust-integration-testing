//! Continuously measure the eCO2 and eTVOC in the air and print it to an SSD1306 OLED display.
//! This uses constant temperature and humidity rather than measuring. 
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
//!   (See note about possible need to update firmware.)
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> CCS811 <-> Display
//! GND  <-> GND    <-> GND
//! 3.3V <-> VCC    <-> VDD
//! PB8  <-> SCL    <-> SCL  PB8 with bluepill. Check src/i2c_led_delay.rs for other devices.
//! PB9  <-> SDA    <-> SDA  PB9 with bluepill. Check src/i2c_led_delay.rs for other devices.
//! GND  <-> nWAKE
//! 3.3V <-> RST
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use embedded_ccs811::{prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode, SlaveAddr};

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use heapless::String;
use nb::block;

use core::fmt::Write;
use rtt_target::{rprintln, rtt_init_print};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};  // prelude has DisplaySize128x32,  DisplaySize128x64 

use rust_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("CCS811 example");

    let (i2c, mut led, mut delay) = setup();

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut ccs811 = Ccs811Awake::new(manager.acquire_i2c(), SlaveAddr::default());
    ccs811.software_reset().unwrap();
    delay.delay_ms(10_u16);
    let mut lines: [String<32>; 2] = [String::new(), String::new()];

    let mut ccs811 = ccs811.start_application().ok().unwrap();
    let temperature_c = 25.0;
    let humidity_perc = 60.0;
    ccs811
        .set_environment(temperature_c, humidity_perc)
        .unwrap();
    ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

    let default = AlgorithmResult {
        eco2: 9999,
        etvoc: 9999,
        raw_current: 255,
        raw_voltage: 9999,
    };
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(100_u16, &mut delay);
        delay.delay_ms(100_u16);

        let data = block!(ccs811.data()).unwrap_or(default);

        for line in lines.iter_mut() {
            line.clear();
        }
        write!(lines[0], "eCO2: {} ppm", data.eco2).unwrap();
        write!(lines[1], "eTVOC: {} ppb", data.etvoc).unwrap();
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            //with font 6x10, 12 = 10 high + 2 space
            Text::with_baseline(line, Point::new(0, i as i32 * 12), text_style, Baseline::Top,)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
