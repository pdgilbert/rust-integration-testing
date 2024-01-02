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
use embedded_hal::delay::DelayNs;

use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::pac::{Peripherals};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("AT24C256 example");
    hprintln!("AT24C256 example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2c, _i2c2, mut led, mut delay, _clock) = i2c1_i2c2_led_delay::setup_from_dp(dp);

    let mut eeprom = Eeprom24x::new_24x256(i2c, SlaveAddr::default()); //SlaveAddr::Alternative(true, true, true));
    let memory_address = 0x01;
    hprintln!("program").unwrap();
    eeprom
        .write_page(memory_address, &[0xAB, 0xCD, 0xEF, 0x12])
        .unwrap();
    
    hprintln!("check").unwrap();

    // wait maximum time necessary for write
    delay.delay_ms(5);
    loop {
        let mut data = [0; 4];
        eeprom.read_data(memory_address, &mut data).unwrap();
        if data == [0xAB, 0xCD, 0xEF, 0x12] {
            led.blink(250_u16, &mut delay);
            delay.delay_ms(250);
        }
    }
}
