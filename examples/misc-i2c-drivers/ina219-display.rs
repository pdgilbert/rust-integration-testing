//! Continuously read INA219 current monitor and display on SSD1306 OLED.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.)

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use  ina219::{INA219,}; //INA219_ADDR

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::i2c_led_delay::{setup, LED, };
use embedded_hal::blocking::delay::DelayMs;

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("INA219 example");
    //hprintln!("INA219 example").unwrap();

    let (i2c, mut led, mut delay) = setup();
    
    led.off();
    delay.delay_ms(2000_u16);
    led.blink(1000_u16, &mut delay);

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    //common display sizes are 128x64 and 128x32
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    //hprintln!("display.flush").unwrap();

    //  note that larger font size increases memory and may require building with --release
    //  &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //  &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8 =   4  characters high
    //  &FONT_4X6  128 pixels/ 4 per font =  32  characters wide.  32/6 =  5.3 characters high

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];

    let mut ina = INA219::new(manager.acquire_i2c(), 0x40);
    //hprintln!("let mut ina addr {:?}", INA219_ADDR).unwrap();  // crate's  INA219_ADDR prints as 65

    ina.calibrate(0x0100).unwrap();

    led.blink(3000_u16, &mut delay);

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        led.blink(20_u16, &mut delay);

        lines[0].clear();
        lines[1].clear();
        
        let v = match ina.voltage() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let vs = match ina.shunt_voltage() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let i = match ina.current() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let p = match ina.power() {  // ina indicated power
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };
        
        // power caclulated by P=IV.  
        //If the ina is wired with Vin- to battery and Vin+ to load sign then the
        // display will show "+" for battery charging and "-" for discharging.
        let pc = i as i32 * v as i32;

        write!(lines[0], "V: {}mv Vs: {}mV", v, vs).unwrap();
        write!(lines[1], "I: {}mA P:{}mW [{}mW]", i,  p, pc).unwrap();
        
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
        
        delay.delay_ms(2000_u16);
    }
}

