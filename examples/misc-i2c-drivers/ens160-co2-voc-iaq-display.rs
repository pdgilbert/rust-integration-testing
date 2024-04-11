//! Continuously measure the eCO2 and eVOC in the air and print it to an SSD1306 OLED display.
//! Ssd1306 is on i2c1 and ens160 is on i2c2. There is no bus sharing.
//! This uses constant temperature and humidity for the sensor calibration rather than measuring. 
//! (An ens160+AHT2x board has aht2x, typically aht21 which can be read and used to improve
//! the calculations. See projects/IAQ-ens160aht21-display.rs for an example.)
//! 
//! Regarding sensor power supply issue see
//!  https://community.home-assistant.io/t/issue-with-ens160-aht21-ens160-readings-unavailable/697522/2
//!
//! Compare  ccs811-gas-voc-display  and  aht20-display
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]

//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::{hprintln};

use cortex_m_rt::entry;

/////////////////////   ens
use ens160::{Ens160, AirQualityIndex, ECo2};


/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space
//with font 6x10, VPIX=12 = 10 high + 2 space ; 8X13  VPIX=14 = 13 + 1

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};


use heapless;

/////////////////////  setups

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, DelayNs,};

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("example");
    //hprintln!("ens160-co2-voc-iaq-display example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(dp);

    //    let mut delay_syst = cp.SYST.delay(&clocks); 

    /////////////////////   ssd
    //hprintln!("ssd start").unwrap();

    let interface = I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();
    //display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

    display.clear_buffer();
    Text::with_baseline(
        "Display initialized ...",
        //Point::new(0, i as i32 * VPIX),
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    /////////////////////   ens

    //hprintln!("ens start").unwrap();
    let mut ens = Ens160::new(i2c2, 0x53);  //0x52 Ens160,  0x53 Ens160+Aht21
    let _z = ens.reset();

   // match z {
   //         Ok(v)   =>  {hprintln!("v:{:?}, ",v).unwrap()},
   //         Err(e)  =>  {hprintln!(" ens.reset() Error {:?}. ", e).unwrap(); 
   //                      //panic!("Error reading"),
   //                      //(25, 40)  //supply default values
   //                     },
   // };

    delay.delay_ms(250);
    let _z = ens.operational();

   // match z {
   //         Ok(v)   =>  {hprintln!("v:{:?}, ",v).unwrap()},
   //         Err(e)  =>  {hprintln!(" ens.operational() Error {:?}. ", e).unwrap(); 
   //                      //panic!("Error reading"),
   //                      //(25, 40)  //supply default values
   //                     },
   // };


    ///////////////////// initialize loop variables

    delay.delay_ms(1000);

    let mut lines: [heapless::String<32>; 4] = [heapless::String::new(), heapless::String::new(),
                                                heapless::String::new(), heapless::String::new(), ];


    // if these are initialized they can be reset in the loop and values used after t he loop
    let mut tvoc: u16;
    let mut eco2: ECo2;
    let mut aqi1: AirQualityIndex ;
    let mut aqi2: AirQualityIndex ;
    #[allow(unused_assignments)]
    let mut temp: i16 = -32767;    // initialize with imposible values which may never be used
    #[allow(unused_assignments)]
    let mut humd: u16 =  20000;    // initialize with imposible values which may never be used

    /////////////////////    measure and display in loop

    //hprintln!("loop start").unwrap();
    loop {
        // Blink LED to check that everything is actually running.
        // If the LED is off, something went wrong.
        led.blink(100_u16, &mut delay);
        delay.delay_ms(100);

        if let Ok(status) = ens.status() {
            if status.data_is_ready() {
                tvoc = ens.tvoc().unwrap();
                eco2 = ens.eco2().unwrap();
                aqi1 = AirQualityIndex::try_from(eco2).unwrap();  // from eco2
                aqi2 = ens.air_quality_index().unwrap();  // directly
                (temp, humd) = ens.temp_and_hum().unwrap();
                //hprintln!("tvoc:{:?}, ", tvoc).unwrap();
                //hprintln!("eco2:{:?}, ", eco2).unwrap();
                //hprintln!("aqi1:{:?}, ", aqi1).unwrap();
                //hprintln!("aqi2:{:?}, ", aqi2).unwrap();
                //hprintln!("temp:{:?}, ", temp).unwrap();
                //hprintln!("humd:{:?}, ", humd).unwrap();

    // temp_and_hum(&mut self) -> Result<(i16, u16), E> 
    //The (temp, humd) units are scaled by 100. For example, a temperature value of 2550 represents 25.50
    // and a humidity value of 5025 represents 50.25% RH.

                for line in lines.iter_mut() {line.clear();}
                
                // The TVOC level is expressed in parts per billion (ppb) in the range 0-65000.
                // The eCO2 level is expressed in parts per million (ppm) in the range 400-65000.
                write!(lines[0], "TVOC {}ppb {:?}ppm", tvoc, eco2).unwrap();
                write!(lines[1], "AQI 1: {:?}  ", aqi1).unwrap();
                write!(lines[2], "AQI 2: {:?}  ", aqi2).unwrap();
                write!(lines[3], "{:?} deg  {:?} %RH", temp, humd).unwrap();  // /100

                // note line[3] is off the sceen with 6x10, VPIX= 12
                display.clear_buffer();
                for i in 0..lines.len() {
                    //with font 6x10, VPIX= 12 = 10 high + 2 space
                    Text::with_baseline(
                       &lines[i], Point::new(0, i as i32 * VPIX), text_style, Baseline::Top,
                    ).draw(&mut display).unwrap();
                }
                display.flush().unwrap();
            }
        }
    
        delay.delay_ms(5000);
        //hprintln!("loop end").unwrap();
    }
    //let i2c = ens.release(); // destruct driver to re-use bus
}
