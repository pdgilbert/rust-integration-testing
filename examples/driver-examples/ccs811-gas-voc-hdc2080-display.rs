//! Continuously measure the eCO2 and eTVOC in the air and print it to an
//! SSD1306 OLED display.
//! In order to compensate for the ambient temperature and humidity, an HDC2080
//! sensor is used.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> CCS811 <-> HDC2080 <-> Display
//! GND  <-> GND    <-> GND     <-> GND
//! 3.3V <-> VCC    <-> VCC     <-> VDD
//! PB8  <-> SCL    <-> SCL     <-> SCL  PB8 with bluepill. Check src/i2c_led_delay.rs for other devices.
//! PB9  <-> SDA    <-> SDA     <-> SDA  PB9 with bluepill. Check src/i2c_led_delay.rs for other devices.
//! GND  <-> nWAKE
//! 3.3V <-> RST
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use embedded_ccs811::{
    prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode, SlaveAddr as Ccs811SlaveAddr,
};
use hdc20xx::{Hdc20xx, SlaveAddr as Hdc20xxSlaveAddr};

use cortex_m_rt::entry;
//use embedded_hal::blocking::delay::DelayMs;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;

use heapless::String;
use nb::block;

use core::fmt::Write;
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
    rprintln!("CCS811/HDC2080 example");

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

    let mut hdc2080 = Hdc20xx::new(manager.acquire_i2c(), Hdc20xxSlaveAddr::default());
    let mut ccs811 = Ccs811Awake::new(manager.acquire_i2c(), Ccs811SlaveAddr::default());
    ccs811.software_reset().unwrap();
    delay.delay_ms(10_u16);
    let mut lines: [String<32>; 4] = [String::new(), String::new(), String::new(), String::new()];

    let mut ccs811 = ccs811.start_application().ok().unwrap();
    let mut env = block!(hdc2080.read()).unwrap();
    ccs811
        .set_environment(env.temperature, env.humidity.unwrap_or(0.0))
        .unwrap();
    ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

    let default = AlgorithmResult {
        eco2: 9999,
        etvoc: 9999,
        raw_current: 255,
        raw_voltage: 9999,
    };

    let mut counter = 0;
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(500_u16, &mut delay);
        delay.delay_ms(500_u16);

        let data = block!(ccs811.data()).unwrap_or(default);

        counter += 1;
        if counter > 10 {
            counter = 0;

            env = block!(hdc2080.read()).unwrap();
            ccs811
                .set_environment(env.temperature, env.humidity.unwrap_or(0.0))
                .unwrap();
        }

        for i in 0..4 {
            lines[i].clear();
        }
        write!(lines[0], "eCO2: {}", data.eco2).unwrap();
        write!(lines[1], "eTVOC: {}", data.etvoc).unwrap();
        write!(lines[2], "Temp: {:.2}ºC", env.temperature).unwrap();
        write!(lines[3], "Humidity: {:.2}%", env.humidity.unwrap_or(0.0)).unwrap();
        display.clear();
        for (i, line) in lines.iter().enumerate() {
            Text::new(line, Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
