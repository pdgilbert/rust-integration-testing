//! Continuously measure the eCO2 and eVOC in the air and print it to an SSD1306 OLED display.
//! Ssd1306 is on i2c1 and ens160 is on i2c2. There is no bus sharing.
//! This uses constant temperature and humidity rather than measuring. ??? board has aht21 ???
//!
//! Compare  ccs811-gas-voc-display.
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]

//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::{hprintln};

use cortex_m_rt::entry;

/////////////////////   ens
use ens160::{Ens160, AirQualityIndex, ECo2};


/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};


/////////////////////   hals

use embedded_hal::{
   //i2c::I2c as I2cTrait,
   delay::DelayNs,
};

use heapless::String;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

use hal::{
   pac::{Peripherals},
};

/////////////////////  

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("example");
    hprintln!("ens160-co2-voc-iaq-displayexample").unwrap();

    let dp = Peripherals::take().unwrap();
//    let cp = CorePeripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay, _clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);

//    let mut delay_syst = cp.SYST.delay(&clocks); // ens160 takes one delay

    /////////////////////   ssd

    let interface = I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();
    //display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        "Display initialized ...",
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    /////////////////////   ens
    let mut ens = Ens160::new(i2c2, 0x52);  //0x52 (default?) and 0x53 
    ens.reset().unwrap();
    delay.delay_ms(250);
    ens.operational().unwrap();

    delay.delay_ms(250);
    let mut lines: [String<32>; 2] = [String::new(), String::new()];

    /////////////////////    measure and display in loop

    // if these are initialized they can be reset in the loop and values used after t he loop
    let mut tvoc: u16;
    let mut eco2: ECo2;
    let mut aqi1: AirQualityIndex ;
    let mut aqi2: AirQualityIndex ;

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

                for line in lines.iter_mut() {line.clear();}
                write!(lines[0], "eTVOC: {} ppb  eco2: {:?}", tvoc, eco2).unwrap();
                write!(lines[1], "air qual. index 1: {:?}   2: {:?}", aqi1, aqi2).unwrap();
                display.clear_buffer();
                for (i, line) in lines.iter().enumerate() {
                    //with font 6x10, 12 = 10 high + 2 space
                    Text::with_baseline(line, Point::new(0, i as i32 * VPIX), text_style, Baseline::Top,)
                        .draw(&mut display)
                    .unwrap();
                }
                display.flush().unwrap();
            }
        }
    
        delay.delay_ms(5000);
    }
    //let i2c = ens.release(); // destruct driver to re-use bus
}
