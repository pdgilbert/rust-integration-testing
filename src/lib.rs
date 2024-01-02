// The library code is only setup functions:
//   See  src/lora_spi_gps_usart.rs
//   See examples for most testing.

#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


pub mod stm32xxx_as_hal;   

pub mod monoclock;// TRY TO GET RID OF THIS
pub mod prelude;// TRY TO GET RID OF THIS

//// pub mod lora_spi_gps_usart; 

//  crate si4703 does not compile with eh-1.0.0 this should probably removed and code put in examples.
#[cfg(feature = "stm32f4xx")] //si4703
pub mod i2c_led_delay_buttons_stcint;

pub mod onewire_i2c_led; // TRY TO GET RID OF THIS or consolidate with dht_i2c_led_usart

pub mod i2c1_i2c2_led_delay;

pub mod i2c1_i2c2_led;  // TRY TO GET RID OF THIS ?
////pub mod dht_i2c_led_delay; // TRY TO GET RID OF THIS
pub mod dht_i2c_led_usart;    // OR THIS

pub mod i2c_led;   

pub mod dp;
pub mod cp;
pub mod delay;
pub mod delay_syst;
pub mod alt_delay;
pub mod led;
pub mod i2c;
////pub mod usart; separating this does not simplify much
pub mod dht; // TRY TO GET RID OF THIS and use hal as in example dht
pub mod onewire;
////pub mod spi; move problems need to be figured out for this

// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
