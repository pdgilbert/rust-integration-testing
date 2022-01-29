// The library code is only setup functions:
//   See  src/lora_spi_gps_usart.rs
//   See examples for most testing.

#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub mod i2c_led_delay;
pub mod led;
pub mod i2c;
// temp rm for eh-1   pub mod lora_spi_gps_usart;
pub mod alt_delay;

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
