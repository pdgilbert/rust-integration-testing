// The library code is only setup functions:
//   See  src/lora_spi_gps_usart.rs
//   See examples for most testing.

#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub mod monoclock;
//// pub mod lora_spi_gps_usart; 
//pub mod i2c_led_delay_buttons_stcint;
pub mod onewire_i2c_led; // TRY TO GET RID OF THIS
pub mod i2c1_i2c2_led;  // TRY TO GET RID OF THIS
////pub mod dht_i2c_led_delay;
pub mod dht_i2c_led_usart; // TRY TO GET RID OF THIS

pub mod i2c_led;   // previously i2c_led_delay // TRY TO GET RID OF THIS

pub mod dp;
pub mod cp;
pub mod delay;
pub mod delay_syst;
pub mod alt_delay;
pub mod led;
pub mod i2c;
////pub mod usart; separating this does not simplify much
pub mod dht;
pub mod onewire;
////pub mod spi; move problems need to be figured out for this

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
