//! Feb 2023. tested to work on both bluepill and blackpill stm32f401
//!                         with display on i2c1 and sensor on i2c2
//!                         and with display on i2c2 and sensor on i21
//!                         and with shared bus on i2c1 or i2c2
//!                         using both probe and battery power.
//!       on blackpil with battery power the NRST button needs to be pressed
//!                      after power up if the bootloader has not been bypassed.
//! 
//! Continuously read temperature from HDC1080 and display on SSD1306 OLED.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.  Using VCC  3.3v.)

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use embedded_hdc1080_rs::{Hdc1080}; 
//use hdc1080::{ Hdc1080, SlaveAddr };   //this one needs std

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::setups::{
    setup_i2c1_i2c2_led_delay_using_dp, Peripherals, DelayNs};


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, _led, mut delay) = setup_i2c1_i2c2_led_delay_using_dp(dp);

    //let manager = shared_bus::BusManagerSimple::new(i2c1); 
    //let interface = I2CDisplayInterface::new(manager.acquire_i2c());
    let interface = I2CDisplayInterface::new(i2c2);

    //common display sizes are 128x64 and 128x32
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];

    Text::with_baseline(   "hdc1080-display", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();
    delay.delay_ms(2000_u32);

    // Start the sensor.
    //let mut sensor = Hdc1080::new(manager.acquire_i2c(), delay).unwrap();
    let mut sensor = Hdc1080::new(i2c1, delay).unwrap();

    sensor.init().unwrap();
    //delay.delay_ms(500_u16);

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        //led.blink(20_u16, &mut delay); //need another delay

        // Read humidity and temperature.
        let (temperature, h) = sensor.read().unwrap();

        lines[0].clear();
        lines[1].clear();
        write!(lines[0], "temperature: {} C", temperature).unwrap();
        write!(lines[1], "relative humidity: {0}%", h).unwrap();
        
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::with_baseline(
                line,
                Point::new(0, i as i32 * 16),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .unwrap();
        }
        display.flush().unwrap();
    }
}
