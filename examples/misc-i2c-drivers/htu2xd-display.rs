//! Continuously read temperature from htu21D sensor and display on SSD1306 OLED.
//! Compiles but is too large to load on bluepill.
//!
//!  The setup() functions make the application code common. They are in src/.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.

//! Compare examples htu21D_rtic, aht10-display, aht10_rtic, dht_rtic, oled_dht, and blink_rtic.
//! Following https://github.com/samcrow/HTU2XD/blob/master/src/lib.rs


#![deny(unsafe_code)]
#![no_std]
#![no_main]

use htu2xd::{Htu2xd, Reading};   //, Resolution

//use embedded_hal::blocking::delay::DelayUs;

// use embedded_hal::blocking::i2c::{Read, Write, WriteRead};


#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20 as FONT, MonoTextStyleBuilder},   //FONT_5X8   FONT_10X20
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::setups::{
    setup_i2c1_i2c2_led_delay_using_dp, Peripherals, LED, DelayMs};


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("htu21D-display example");
    //hprintln!("htu21D-display example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2c1, _i2c2, mut led, mut delay) = setup_i2c1_i2c2_led_delay_using_dp(dp);

    led.blink(1000_u16, &mut delay); // Blink LED to indicate setup finished.

    let manager = shared_bus::BusManagerSimple::new(i2c1);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    //common display sizes are 128x64 and 128x32
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];

    Text::with_baseline(    "HTU2XD-display",Point::zero(), text_style, Baseline::Top )
        .draw(&mut display).unwrap();
    display.flush().unwrap();
    delay.delay_ms(2000u32);

    led.blink(500_u16, &mut delay); // Blink LED to indicate Ssd1306 initialized.

    // Start the sensor.
    let mut htu    = Htu2xd::new();
    let mut htu_ch = manager.acquire_i2c();

    //htu.soft_reset(i2c)?;
    htu.soft_reset(&mut htu_ch).expect("sensor reset failed");

    // Wait for the reset to finish
    delay.delay_ms(15u32);

    //    .read_user_register() dos not return and changes something that requires sensot power off/on.
    //    let mut register = htu.read_user_register(&mut htu_ch).expect("htu.read_user_register failed");
    //    register.set_resolution(Resolution::Humidity10Temperature13);   //.expect("set_resolution failed");
    //    htu.write_user_register(&mut htu_ch, register).expect("write_user_register failed");

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        led.blink(10_u16, &mut delay);

        lines[0].clear();
        lines[1].clear();

        let z = htu.read_temperature_blocking(&mut htu_ch);
        //hprintln!("{:?}", z).unwrap();
        //  there is a double wrapping:  Ok(Ok(Temperature(24624)))

        match z {
            Ok(Reading::Ok(t))     => {//hprintln!("{} deg C", t.as_degrees_celsius()).unwrap();
                                  write!(lines[0], "  {:.1} C", t.as_degrees_celsius()).unwrap(); },

            Ok(Reading::ErrorLow)  => {//hprintln!("Error or off-scale low").unwrap();
                                  write!(lines[0], "Error or off-scale low").unwrap(); },

            Ok(Reading::ErrorHigh) => {//hprintln!("Error or off-scale high").unwrap();
                                  write!(lines[0], "Error or off-scale high").unwrap(); },

            Err(_)                 => {//hprintln!("Error reading temperature").unwrap();
                                  write!(lines[0], "Error reading temperature").unwrap(); },
        }

        let z = htu.read_humidity_blocking(&mut htu_ch);

        match z {
            Ok(Reading::Ok(t))     => {//hprintln!("{}% RH", t.as_percent_relative()).unwrap();
                                       write!(lines[1], "  {:.0}% RH", t.as_percent_relative()).unwrap();},

            Ok(Reading::ErrorLow)  => {//hprintln!("Error or off-scale low").unwrap();
                                  write!(lines[1], "humidity off-scale low").unwrap(); },

            Ok(Reading::ErrorHigh) => {//hprintln!("Error or humidity off-scale high").unwrap();
                                  write!(lines[1], "humidity off-scale high").unwrap(); },

            Err(_)                 => {//hprintln!("Error reading humidity").unwrap();
                                  write!(lines[1], "Error reading humidity").unwrap(); },
        }

     
        display.clear();
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
    delay.delay_ms(1000u32);
    }
}
