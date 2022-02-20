// WORK IN PROGRESS
//! Measure temperature with 10k thermistor sensor and display on SSD1306 OLED display.
//! See misc/temperature.rs and misc/temperature_display.rs forother sensors and 
//! the internal mcu temperature measurement.
//! 
//! Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display.
//!
//! The 10K thermistor voltage is measured with      analog/digital converter.
//!
//! The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!
//! On "BluePill" (stm32f1xx_hal) the I2C1 connections are SDA on PB9, SCL on PB8. ADC as below.
//! See src/i2c_led_delay.rs for settings on other boards. The section of setup() corresponding 
//! to the HAL setting specifies details of pin connections.
//! If 3.3v is supplied through BluePill from 5v BEWARE of regulator limit.
//! 
//! The SSD1306 OLED display connects to the I2C bus: VCC  (or VDD) to 3.3v, and also to GND, SDA, and SCL. 
//! 
//! The 10K thermistor connects to 3.3v and is in series with a 10K fixed resistor which connects to GND.
//! (see for example https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html
//!  or  https://learn.adafruit.com/thermistor/using-a-thermistor
//! Voltage at the thermistor to fixed resistor connection is measured on the ADC pin.  


#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
//use embedded_hal::adc::OneShot;
//use nb::block;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
// DisplaySize128x32:
//    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
//    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
//    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high

const DISPLAY_LINES: usize = 3; 

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};

use rust_integration_testing_of_examples::adc_i2c_led_delay::{setup, LED, ReadAdc};

fn show_display<S>(
    thermistor: u32,
    temp: u32,
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; DISPLAY_LINES] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // it is now possible to use \n in place of separate writes, with one line rather than vector.
    write!(lines[0], "thermistor{:3} C", thermistor).unwrap();
    write!(lines[1], "temp:    {:5} C", temp).unwrap();

    disp.clear();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 12), //using font 6x10, 12 = 10 high + 2 space
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
    //rprintln!("therm10k_display example");
    hprintln!("therm10k_display example").unwrap();

    let (mut sens, i2c, mut led, mut delay) = setup();

    led.blink(500_u16, &mut delay);  // to confirm startup

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();


    loop {
        // Blink LED  to check that loop is actually running.
        led.blink(50_u16, &mut delay);

        // Read adc
        let thermistor = sens.read_mv();
        //let thermistor = block!(sens.read_mv()).unwrap_or(8091) * 5;  //FIX
        let temp = thermistor;  //FIX formula

    hprintln!("values read").unwrap();
    hprintln!("values  {} {} ", thermistor, temp).unwrap();

        show_display(thermistor, temp, text_style, &mut display);

        delay.delay_ms(5000_u16);
    }
}
