//!   NOT HARDWARE TESTED SINCE EMBEDDED-HAL V1.0.0 CHANGES
//!
//!  Examples dht, dht_rtic, and oled_dht are similar and might be consolidated sometime.
//!  
//!  Measure the temperature and humidity from a DHT11 or DHT22 on data pin (A8) and display on OLED with i2c.
//!  (Specify feature "dht22"for DHT22).
//!  Compare example dht_rtic for similar capability.
//!  See example dht for more details, and also compare examples oled_gps, and temperature_display.
//!    [The DHT data pin is connected to the MCU pin PA8 in most (all) cases. ]
//!  The i2c is using src/setup.rs which has pin details.
//!    [ oled is on i2c1 using scl on pb8 and  sda on pb9 in many cases, including blackpill ]
//!  Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display
//!  Note that '--release' is needed when doing a run test on actual hardware. Otherwise
//!  code is too slow for the timeout set in the crate and run gives 'Error Timeout'.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use embedded_hal::delay::DelayNs;

//use nb::block;
use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::{blocking::read, Reading};
#[cfg(feature = "dht22")]
use dht_sensor::dht22::{blocking::read, Reading};
//use dht_sensor::*;

// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
// DisplaySize128x32:
//    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
//    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
//    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
//    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
//    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
//    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high

use embedded_graphics::{
    //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode,
                  prelude::DisplaySize128x32 as DISPLAYSIZE };

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED};



fn show_display<S>(
    temperature: i8,
    relative_humidity: u8,
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 1] = [
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // It is possible to use \n in place of separate writes, with one line rather than vector.

    // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
    // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
    // the line commented out!! The test_txt is taken from 
    //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
    
    //let test_text  = "¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
    //   degree symbol "°" is about                  ^^ here 
    
    write!(lines[0], "{:3}°C {:3}% RH", temperature, relative_humidity).unwrap();
 
    disp.clear_buffer();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("oled_dht example");
    //hprintln!("oled_dht example").unwrap();

    let dp = Peripherals::take().unwrap();
    let (mut dht, i2c, mut led, mut delay) = setup::pin_i2c_led_delay_from_dp(dp);

    led.blink(500_u16, &mut delay);  // to confirm startup

    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    loop {
        // Blink LED to check that everything is actually running.
        led.blink(50_u16, &mut delay);

        match read(&mut delay, &mut dht) {
            Ok(Reading {
                temperature,
                relative_humidity,}) 
               => {//hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                   show_display(temperature, relative_humidity, text_style, &mut display)},
            Err(_e) 
               =>  {//hprintln!("Error {:?}", e).unwrap(); 
                    panic!("Error reading DHT")},
        }

        // (Delay at least 500ms before re-polling DHT, 1 second or more advised)
        delay.delay_ms(2000); // Delay 2 seconds
    }
}
