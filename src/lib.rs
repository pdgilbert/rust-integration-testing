// The library code is only setup functions:
//   See  src/lora_spi_gps_usart.rs
//   See examples for most testing.

#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub mod dht_i2c_led_delay;
pub mod adc_i2c_led_delay;
pub mod i2c_led_delay;

pub mod dp;
pub mod delay;
pub mod led;
pub mod i2c;
//pub mod spi; move problems need to be figured out for this
pub mod alt_delay;

// this should not be commented out but if compiling it fails then other tests do not run.
//pub mod lora_spi_gps_usart;

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
