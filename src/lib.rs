// The library code is only setup functions:

#![no_std]

#![feature(type_alias_impl_trait)]   //  for impl Trait` in type aliases is unstable

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


pub mod stm32xxx_as_hal;   

pub mod monoclock;

pub mod delay;
pub mod delay_syst;
pub mod alt_delay;
pub mod led;
pub mod i2c;

pub mod usart;

pub mod i2c1_i2c2_led_delay;

pub mod  setup;

pub mod lora; 

//pub mod spi; move problems need to be figured out for this

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
