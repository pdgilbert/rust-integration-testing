//! Stores the date and time on a DS1307 real-time clock (RTC).
//! Then reads the date and time repeatedly blink LED 0 for 30 seconds.
//!
//! Introductory blog post here:
//! https://blog.eldruin.com/ds1307-real-time-clock-rtc-driver-in-rust/
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs. 
//!  The specific function used will depend on the HAL setting (see README.md). 
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP  <-> DS1307
//! GND <-> GND
//! +5V <-> +5V
//! PB8 <-> SCL
//! PB9 <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use ds1307::{Ds1307, NaiveDate, Rtcc};

use cortex_m_rt::entry;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;

use rtt_target::{rprintln, rtt_init_print};

use hal_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("DS1307 example");

    let (i2c, mut led, mut delay) = setup();

    let mut rtc = Ds1307::new(i2c);
    let begin = NaiveDate::from_ymd(2020, 5, 2).and_hms(10, 21, 34);
    rtc.set_datetime(&begin).unwrap();
    loop {
        let now = rtc.get_datetime().unwrap();
        if (now - begin).num_seconds() < 30 {
            // this will blink for 30 seconds
            led.blink(250_u16, &mut delay);
            delay.delay_ms(250_u16);
        }
    }
}
