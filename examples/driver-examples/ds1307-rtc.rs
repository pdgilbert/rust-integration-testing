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

use ds1307::{Ds1307, NaiveDate, DateTimeAccess};

use embedded_hal::delay::DelayNs;
use cortex_m_rt::entry;

use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::setup;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::pac::{Peripherals};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("DS1307 real time clock example");
    hprintln!("DS1307 real time clock example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2c, mut led, mut delay) = setup::i2c_led_delay_from_dp(dp);

    hprintln!("rtc").unwrap();
    let mut rtc = Ds1307::new(i2c);
    hprintln!("begin").unwrap();
    let begin = NaiveDate::from_ymd_opt(2020, 5, 2).expect("from_ymd failed").and_hms_opt(10, 21, 34).expect("hms failed");
    hprintln!("rtc.set_datetime").unwrap();
    rtc.set_datetime(&begin).unwrap();
    hprintln!("loop").unwrap();
    loop {
        let now = rtc.datetime().unwrap();
        hprintln!("now {}", now).unwrap();
        if (now - begin).num_seconds() < 30 {
            // this will blink for 30 seconds
            hprintln!("blink").unwrap();
            led.blink(250_u16, &mut delay);
            hprintln!("delay").unwrap();
            delay.delay_ms(250);
        }
    }
}
