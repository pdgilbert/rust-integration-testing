//! Continuously read VL53LOX Time of Flight IR laser distance sensor and display on SSD1306 OLED.
//!
//!  Use caution, protect eyes from the laser !!
//!
//! Following example  https://github.com/copterust/proving-ground/blob/master/vl53l0x/main.rs
//!  https://www.st.com/resource/en/datasheet/vl53l0x.pdf
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.  Using VCC  3.3v.)

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use vl53l0x; 

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::dp::{Peripherals};
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;
use rust_integration_testing_of_examples::led::{LED};


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("VL53L0X example");
    //hprintln!("VL53L0X example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2c, _i2c2, mut led, mut delay, _clocks) = i2c1_i2c2_led_delay::setup(dp);

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    //common display sizes are 128x64 and 128x32
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    //  note that larger font size increases memory and may require building with --release
    //  &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //  &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8 =   4  characters high
    //  &FONT_4X6  128 pixels/ 4 per font =  32  characters wide.  32/6 =  5.3 characters high

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];

    // Start the sensor. with default configuration
    let mut tof = vl53l0x::VL53L0x::new(manager.acquire_i2c()).expect("vl");
    
    tof.set_measurement_timing_budget(200000).expect("timbudg");
    tof.start_continuous(0).expect("start cont");

     loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        led.blink(20_u16, &mut delay);

        lines[0].clear();
        lines[1].clear();
        match tof.read_range_continuous_millimeters_blocking() {
            Ok(meas) => write!(lines[0], "vl: {} mm", meas).unwrap(),
            Err(e)   => write!(lines[0], "Err0r: {:?}", e).unwrap()
        };
        
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

