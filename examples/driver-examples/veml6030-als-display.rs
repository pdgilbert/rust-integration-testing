//! Continuously measure the ambient light sensor data
//! and print it to an SSD1306 OLED display in lux.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/veml6030-ambient-light-sensor-driver-in-rust/
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

use veml6030::{SlaveAddr, Veml6030};

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED};


#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("VEML6030 example");

    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(dp);

    let interface = I2CDisplayInterface::new(i2c1);
    //let mut disp: GraphicsMode<_,_> = Builder::new().connect(interface).into();
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut sensor = Veml6030::new(i2c2, SlaveAddr::default());
    sensor.enable().unwrap();

    let mut buffer: heapless::String<64> = heapless::String::new();
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        // If there is an error, it will print -1.0
        let lux = sensor.read_lux().unwrap_or(-1.0);

        buffer.clear();
        write!(buffer, "lux {:.2}", lux).unwrap();
        display.clear_buffer();
        Text::new(&buffer, Point::zero(), text_style)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
    }
}
