// The library code is only setup functions:
//   See  src/lora_spi_gps_usart.rs
//   See examples for most testing.

#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

// pub mod lora_spi_gps_usart; 
pub mod onewire_i2c_led_delay;
pub mod i2c1_i2c2_led_delay;
//pub mod dht_i2c_led_delay;
pub mod dht_i2c_led_usart_delay;
pub mod adc_i2c_led_delay;
pub mod i2c_led_delay;

pub mod dp;
pub mod delay;
pub mod led;
pub mod i2c;
pub mod dht;
pub mod onewire;
//pub mod spi; move problems need to be figured out for this
pub mod alt_delay;

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
