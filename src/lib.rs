// The library code is only setup functions:

#![no_std]

#![feature(type_alias_impl_trait)]   //  for impl Trait` in type aliases is unstable

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


pub mod alt_delay;

pub mod led;   // has trait and default impl
pub mod lora;  // has structures and constants

pub mod  setup;

// Note, if feature gate is not set then compiling for the module will be 
//  attempted, and features gates are not set in the module, making a mess.

//#[cfg(not(any(feature="stm32f0xx", feature="stm32f1xx", feature="stm32f3xx", feature="stm32f4xx", feature="stm32f7xx", feature="stm32g0xx", feature="stm32g4xx", feature="stm32h7xx",  feature="stm32l0xx",  feature="stm32l1xx", feature="stm32l4xx")))]
// hal featuremust be set

#[cfg(feature = "stm32f0xx")]
pub mod  setup_all_stm32f0xx;

#[cfg(feature = "stm32f1xx")]
pub mod  setup_all_stm32f1xx;

#[cfg(feature = "stm32f3xx")]
pub mod  setup_all_stm32f3xx;

#[cfg(feature = "stm32f4xx")]
pub mod  setup_all_stm32f4xx;

#[cfg(feature = "stm32f7xx")]
pub mod  setup_all_stm32f7xx;

#[cfg(feature = "stm32g0xx")]
pub mod  setup_all_stm32g0xx;

#[cfg(feature = "stm32g4xx")]
pub mod  setup_all_stm32g4xx;

#[cfg(feature = "stm32h7xx")]
pub mod  setup_all_stm32h7xx;

#[cfg(feature = "stm32l0xx")]
pub mod  setup_all_stm3210xx;

#[cfg(feature = "stm32l1xx")]
pub mod  setup_all_stm3211xx;

#[cfg(feature = "stm32l4xx")]
pub mod  setup_all_stm3214xx;


// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
