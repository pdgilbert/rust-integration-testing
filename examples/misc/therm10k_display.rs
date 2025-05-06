// THIS NEEDS WORK. compare projects/temperature-display.rs
// NOTE THAT impl DelayNs does not work dht11::read as of Feb 2024.
//    It still needs DelayUs which is called DelayUsType below.

//! Measure temperature with 10k thermistor sensor (NTC 3950 10k thermistors probe) and temperature and
//! humidity from a DHT-11 (or DHT-22) sensor. Display on SSD1306 OLED display.
//! 
//! One side of the thermistor is connected to GND and other side to adc pin and also through
//! a 10k sresistor to VCC. That makes the max voltage about VCC/2, so about 2.5v when VCC is 5v.
//! and 1.6v when vcc is 3.2v. This is convenient for adc pins that are not 5v tolerant.
//! This means the voltage varies inversely compared to connecting throught the resistor to GND 
//! as is sometimes done. (Since NTC resistance goes down as temperature goes up, this means
//! higher temperature gives higher voltage measurement.) Regarding NTC thermistors see, 
//! for example, https://eepower.com/resistor-guide/resistor-types/ntc-thermistor/#
//!
//! If 3.3v is supplied through BluePill regulator from 5v USB probe BEWARE of regulator current limit.
//! Some places it is claimed that when the limit is exceeded then 5v is supplied but mine failed 
//! by dropping voltage to 2.8v when the SSD1306, 10k thermistor and DHT-11 were on the 3.3v using 
//! USB power rather than battery. It ran but 10K temperature accuracy is questionable.
//!
//! See setup() functions for thepin settings on various boards.
//! 
//! See misc/temperature.rs and misc/temperature_display.rs for other sensors and 
//! the internal mcu temperature measurement.
//! 
//! Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display.
//! 
//! The SSD1306 OLED display connects to the I2C bus: VCC  (or VDD) to 3.3v, and also to GND, SDA, and SCL. 
//! 
//! Voltage at the thermistor to fixed resistor connection is measured on the ADC pin.
//! The choice of 10k series resistor in the voltage divider ...
//! Values in the temperature calculation below are very rough based on my extremely crude calibration
//! effort, but see for example 
//!     https://www.mathscinotes.com/2014/05/yet-another-thermistor-discussion/
//!  or https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html
//!  or  https://learn.adafruit.com/thermistor/using-a-thermistor


// regarding adc use see
// https://www.st.com/resource/en/application_note/cd00258017-stm32s-adc-modes-and-their-applications-stmicroelectronics.pdf

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;

//use embedded_hal::adc::OneShot;
//use nb::block;

use core::fmt::Write;

// Note hprintln will not run without an ST-link probe (eg. not on battery power)
// The use statement does not need to be removed, but it will be unused and causes warnings.
//use cortex_m_semihosting::hprintln;
//use rtt_target::{rprintln, rtt_init_print};

const DISPLAY_LINES: usize = 2; 

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::{blocking::read, Reading};
#[cfg(feature = "dht22")]
use dht_sensor::dht22::{blocking::read, Reading};
//use dht_sensor::Delay;  // trait, whereas timer::Delay is a type does not yet use DelayNs

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, DelayNs, ReadAdc,};


fn show_display<S>(
    mv: u32,
    temp: i64,
    dht_temp: i8, 
    dht_humidity: u8, 
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; DISPLAY_LINES] = [
        heapless::String::new(),
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // it is now possible to use \n in place of separate writes, with one line rather than vector.
    //write!(lines[0], "10k {:4}mV{:3}°C", mv, temp).unwrap();
    write!(lines[0], "10k {:4}mV{:3}C", mv, temp).unwrap();
    write!(lines[1], "dht{:4}%RH{:3}C", dht_humidity, dht_temp).unwrap();

    disp.clear_buffer();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            //using font 6x10, 10 high + 2 space = 12
            //using font 8X13, 13 high + 2 space = 15       
            Point::new(0, i as i32 * 15),
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
    //hprintln!("therm10k_display example").unwrap();

    let dp = Peripherals::take().unwrap();

    // setup needs to return delay because adc for some hals needs delay.
    // setup needs cp for delay from syst.
    // let (mut sens, mut dht, i2c, mut led, mut delay) = setup(dp, cp);
    let ( mut dht, i2c, mut led, mut delay, mut sens) = setup::pin_i2c1_led_delay_adc1_from_dp(dp);

    led.blink(500_u16, &mut delay);  // to confirm startup

    let interface = I2CDisplayInterface::new(i2c);

    //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    // crude linear aproximation
    // t = a - v/b , t degrees C, v in mV, b negative of inverse slope, a includes degrees K to C
    let a = 75i64;
    let b = 40u32; 

    loop {
        // Blink LED  to check that loop is actually running.
        led.blink(50_u16, &mut delay);

        // Read 10k probe on adc
        let mv = sens.read_mv();
        let temp:i64  = a - (mv / b) as i64 ;
        //hprintln!("probe  {}mV  {}C ", mv, temp).unwrap();

        let z = read(&mut delay, &mut dht);
        let (dht_temp, dht_humidity) = match z {
            Ok(Reading {temperature, relative_humidity,})
               =>  {//hprintln!("temperature:{}, humidity:{}, ", temperature, relative_humidity).unwrap();
                    (temperature, relative_humidity)
                   },
            Err(_e) 
               =>  {//hprintln!("dht Error {:?}. Using default temperature:{}, humidity:{}", e, 25, 40).unwrap(); 
                    //panic!("Error reading DHT"),
                    (25, 40)  //supply default values
                   },
        };
 
        show_display(mv, temp, dht_temp, dht_humidity, text_style, &mut display);

        delay.delay_ms(5000);
    }
}
