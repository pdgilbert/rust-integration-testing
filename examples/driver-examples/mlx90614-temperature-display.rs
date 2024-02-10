//! Continuously measure the temperature with an MLX90614 contact-less
//! IR thermopile (thermometer) and print it to an SSD1306 OLED display.
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

use mlx9061x::{Mlx9061x, SlaveAddr};

use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use cortex_m_rt::entry;

use rtt_target::{rprintln, rtt_init_print};

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
    rprintln!("MLX90614 example");

    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay, _clock) = i2c1_i2c2_led_delay::setup_from_dp(dp);

    let interface = I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut sensor = Mlx9061x::new_mlx90614(i2c2, SlaveAddr::default(), 5).unwrap();

    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        let t_obj = sensor.object1_temperature().unwrap_or(-1.0);
        delay.delay_ms(50); // a pause is necessary in between
        let t_a = sensor.ambient_temperature().unwrap_or(-1.0);

        lines[0].clear();
        lines[1].clear();
        write!(lines[0], "Object: {:.2}ºC", t_obj).unwrap();
        write!(lines[1], "Ambient: {:.2}ºC", t_a).unwrap();
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::new(line, Point::new(0, i as i32 * 16), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
    }
}
