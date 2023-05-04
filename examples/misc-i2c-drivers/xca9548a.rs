//! Continuously read temperature from multiple AHT10s and display on SSD1306 OLED.
//! The AHT10s are multiplexed on i2c1 using  xca9548a.
//! 
//! requires two i2c buses. Note also using
//! "https://github.com/andy31415/aht10", branch = "fix_status_check"
//! 
//!  Beware that i2c1parts.i2c2 is the second multiplexed device on i2c1, whereas i2c2 is the MCU's second i2c.
//! 
//!  The setup() functions make the application code common. They are in src/.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.

//! Compare examples aht10-display, aht10_rtic, dht_rtic, oled_dht, and blink_rtic.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use aht10::{AHT10, Humidity, Temperature, Error as aht10Error};

use embedded_hal::blocking::i2c::{Read, WriteRead};   //Write, 
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;  // need trait for switch.write
use xca9548a::{Error as xca9548aError, SlaveAddr, Xca9548a};

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    //mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    //mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    mono_font::{iso_8859_1::FONT_9X15 as FONT, MonoTextStyleBuilder}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::setups::{setup_i2c1_i2c2_led_delays_using_dp, LED, Peripherals, i2cError};


fn show_display<S>(
    s1: Result<(Humidity, Temperature), aht10Error<xca9548aError<i2cError>>>, 
    s2: Result<(Humidity, Temperature), aht10Error<xca9548aError<i2cError>>>, 
    //text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: ssd1306::size::DisplaySize,  //trait
{
   let mut line: heapless::String<64> = heapless::String::new();
         
   match s1 {
        Ok((h,t)) => {hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
                      write!(line, "Sensor 2: {}C\nRH: {:3}%", t.celsius(), h.rh()).unwrap();
                     },
        Err(e)    => {hprintln!("Error {:?}", e).unwrap();
                      write!(line, "Sensor 2 read error. Resetting.code {:?}", e).unwrap();
                     }
    };
         
   match s2 {
        Ok((h,t)) => {hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
                      write!(line, "Sensor 2: {}C\nRH: {:3}%", t.celsius(), h.rh()).unwrap();
                     },
        Err(e)    => {hprintln!("Error {:?}", e).unwrap();
                      write!(line, "Sensor 2 read error. Resetting.code {:?}", e).unwrap();
                     }
    };
   
   //write!(lines[0], "{:.1}°C {:.0}% RH", temperature, relative_humidity).unwrap();
   // write!(lines[1], "{:.1}V {}mA {}mW [{}mW]", v as f32/1000.0, i,  p, pc).unwrap();
  
   show_message(&line, disp);
   ()
}

fn show_message<S>(
    text: &str,   //text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: ssd1306::size::DisplaySize,  //trait
{
   
   // workaround. build here because text_style cannot be shared
   let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

   disp.clear();
   Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
           .draw(&mut *disp)
           .unwrap();

   disp.flush().unwrap();
   ()
}



#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay1, delay2) = setup_i2c1_i2c2_led_delays_using_dp(dp);

    led.off();

    led.blink(2000_u16, &mut delay1); // Blink LED to indicate setup finished.

    let manager = shared_bus::BusManagerSimple::new(i2c2);
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

    Text::with_baseline(   "aht10-display", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();
    //delay1.delay(2000u32);    

    led.blink(500_u16, &mut delay1); // Blink LED to indicate Ssd1306 initialized.
    hprintln!("Text::with_baseline").unwrap();

    // now multiple devices on i2c1 bus
    
    let slave_address = 0b010_0000; // example slave address
    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent

    let mut i2c1switch = Xca9548a::new(i2c1, SlaveAddr::default());

    // Enable channel 0
    i2c1switch.select_channels(0b0000_0001).unwrap();

    // write to device connected to channel 0 using the I2C switch
    if i2c1switch.write(slave_address, &write_data).is_err() {
        hprintln!("Error received!").unwrap();
    }

    // read from device connected to channel 0 using the I2C switch
    let mut read_data = [0; 2];
    if i2c1switch.read(slave_address, &mut read_data).is_err() {
        hprintln!("Error received!").unwrap();
    }

    // write_read from device connected to channel 0 using the I2C switch
    if i2c1switch
        .write_read(slave_address, &write_data, &mut read_data)
        .is_err()
    {
        hprintln!("Error received!").unwrap();
    }

    // Start the sensors.

    // Split the device and pass the virtual I2C devices to AHT10 driver
    let i2c1parts = i2c1switch.split();

    let mut sensor1 = AHT10::new(i2c1parts.i2c1, delay1).expect("sensor failed");
    sensor1.reset().expect("sensor1 reset failed");

    let mut sensor2 = AHT10::new(i2c1parts.i2c2, delay2).expect("sensor2 failed");
    sensor2.reset().expect("sensor2 reset failed");

    loop {
        // Read humidity and temperature.   Sensor 1
        let s1 = sensor1.read();
        if s1.is_err() { sensor1.reset().unwrap()};    //need delay here
        
        // Read humidity and temperature.   Sensor 2
        let s2 = sensor2.read();
        if s2.is_err() { sensor2.reset().unwrap()};    //need delay here

        show_display(s1, s2, &mut display);
    }
}
