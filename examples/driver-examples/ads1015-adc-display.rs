//! Measure voltages with an ADS1015 analog/digital
//! converter and print them to an SSD1306 OLED display.
//! Further explanations about this device and how this example works at
//! https://blog.eldruin.com/ads1x1x-analog-to-digital-converter-driver-in-rust/
//! An image is [here](https://github.com/eldruin/driver-examples/blob/master/media/ads1015-voltage-divider.jpg).
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//!  ```
//!  BP  <-> ADS1015 <-> Display
//!  GND <-> GND     <-> GND
//!  +5V <-> +5V     <-> +5V
//!  PB9 <-> SDA     <-> SDA
//!  PB8 <-> SCL     <-> SCL
//!  ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use ads1x1x::{channel as AdcChannel, Ads1x1x, FullScaleRange, SlaveAddr};

use cortex_m_rt::entry;
use embedded_hal::adc::OneShot;
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
    rprintln!("ADS1015 example");

    let (i2c, mut led, mut delay) = setup();

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut adc = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::default());
    // need to be able to measure [0-5V]
    adc.set_full_scale_range(FullScaleRange::Within6_144V)
        .unwrap();

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

        // Read voltage in all channels
        let values = [
            block!(adc.read(&mut AdcChannel::SingleA0)).unwrap_or(8091),
            block!(adc.read(&mut AdcChannel::SingleA1)).unwrap_or(8091),
            block!(adc.read(&mut AdcChannel::SingleA2)).unwrap_or(8091),
            block!(adc.read(&mut AdcChannel::SingleA3)).unwrap_or(8091),
        ];

        display.clear_buffer();
        for i in 0..values.len() {
            write!(lines[i], "Channel {}: {}", i, values[i]).unwrap();
            Text::new(&lines[i], Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
