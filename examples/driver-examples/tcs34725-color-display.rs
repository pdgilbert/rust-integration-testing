//! Continuously read the color light sensor data and print it to
//! an SSD1306 OLED display.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BlackPill" (stm32f4xx_hal) using I2C1 and I2C2. See file src/i2c.rs for pin settings.
//! ```
//! BP   <-> iAQ-Core-C <-> Display
//! GND  <-> GND        <-> GND
//! 3.3V <-> VCC        <-> VDD
//! PB8  <->            <-> SCL1
//! PB9  <->            <-> SDA1
//! PB10 <-> SCL      
//! PB3  <-> SDA       
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use tcs3472::{AllChannelMeasurement, Tcs3472};

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;

//use rtt_target::{rprintln, rtt_init_print};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED};


#[entry]
fn main() -> ! {
//    rtt_init_print!();
//    rprintln!("TCS34725 example");

    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(dp);

    let interface = I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    let mut sensor = Tcs3472::new(i2c2);
    sensor.enable().unwrap();
    sensor.enable_rgbc().unwrap();
    while !sensor.is_rgbc_status_valid().unwrap() {
        // wait for measurement to be available
        delay.delay_ms(50);
    }
    let mut lines: [heapless::String<32>; 3] = [
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

        write!(lines[0], "Red  Green  Blue").unwrap();
        write!(lines[1], "{}  {}  {}", measurement.red, measurement.green, measurement.blue).unwrap();
        write!(lines[2], "Clear:   {}",  measurement.clear).unwrap();
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::with_baseline(line, Point::new(0, i as i32 * 10), text_style, Baseline::Top,)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
