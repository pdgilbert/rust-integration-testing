//!  Measure the temperature and humidity from a DHT11 sensor and print with hprintln (to
//!  a gdb session).  The DHT11 data pin is connectted to pin A8 on the MCU board and has
//!  a pull up resistor. (18K ohms used in some testing.)
//!  The largest part of this file is the setup() functions used for each hal.
//!  These make the application code common.
//!
//!  Note that '--release' is needed when doing a run test on actual hardware. Otherwise
//!  code is too slow for the timeout set in the crate and run gives 'Error Timeout'.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::Reading;
#[cfg(feature = "dht22")]
use dht_sensor::dht22::Reading;
use dht_sensor::*;

// See dht-sensor git discussion in issues #1  and #2
//https://github.com/michaelbeaumont/dht-sensor/issues/1
//Regarding pulling the pin high to avoid confusing the sensor when initializing.
//Also more in comments in dht-sensor crate file src/lib.rs

use rust_integration_testing_of_examples::dp::{Peripherals};

use rust_integration_testing_of_examples::dht_i2c_led_usart_delay::{
        setup_dht_i2c_led_usart_delay_using_dp, DelayMs};


#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    //let (mut dht, mut delay) = setup(); //dht is usually pa8 in setup functions
    let (mut dht, _i2c, _led, _usart, mut delay) = setup_dht_i2c_led_usart_delay_using_dp(dp);

    hprintln!("Reading sensor...").unwrap();

    // single read before loop for debugging purposes
    //
    //let r = Reading::read(&mut delay, &mut dht);
    //match r {
    //        Ok(Reading {
    //            temperature,
    //            relative_humidity,
    //        }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
    //        Err(e) => hprintln!("Error {:?}", e).unwrap(),
    //}

    loop {
        match Reading::read(&mut delay, &mut dht) {
            Ok(Reading {
                temperature,
                relative_humidity,
            }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
            Err(e) => hprintln!("Error {:?}", e).unwrap(),
        }

        // (Delay at least 500ms before re-polling, 1 second or more advised)
        // Delay 5 seconds
        delay.delay_ms(5000_u16);
    }
}
