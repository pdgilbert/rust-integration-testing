//! Stores some data on an AT24C256C EEPROM.
//! Then reads it again and if it matches, blinks LED 0.
//!
//! Introductory blog post here:
//! https://blog.eldruin.com/24x-serial-eeprom-driver-in-rust/
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs. 
//!  The specific function used will depend on the HAL setting (see README.md). 
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP  <-> AT24C256
//! GND <-> GND
//! +5V <-> +5V
//! PB8 <-> SCL
//! PB9 <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use eeprom24x::{Eeprom24x, SlaveAddr};

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;

use rtt_target::{rprintln, rtt_init_print};

use hal_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("AT24C256 example");

    let (i2c, mut led, mut delay) = setup();

    let mut eeprom = Eeprom24x::new_24x256(i2c, SlaveAddr::Alternative(true, true, true));
    let memory_address = 0x01;
    eeprom
        .write_page(memory_address, &[0xAB, 0xCD, 0xEF, 0x12])
        .unwrap();

    // wait maximum time necessary for write
    delay.delay_ms(5_u16);
    loop {
        let mut data = [0; 4];
        eeprom.read_data(memory_address, &mut data).unwrap();
        if data == [0xAB, 0xCD, 0xEF, 0x12] {
            led.blink(250_u16, &mut delay);
            delay.delay_ms(250_u16);
        }
    }
}
