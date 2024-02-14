// The library code is only setup functions:

#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


pub mod stm32xxx_as_hal;   

pub mod monoclock;

//// pub mod lora_spi_gps_usart; 

pub mod tx1_rx1_tx2_rx2;

pub mod i2c1_i2c2_led_delay;

pub mod  opendrain_i2c_led_usart;

pub mod delay;
pub mod delay_syst;
pub mod alt_delay;
pub mod led;
pub mod i2c;

////pub mod spi; move problems need to be figured out for this

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
