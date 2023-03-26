#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    //pac::TIM3,
};

#[cfg(feature = "stm32f0xx")]
pub type DelayType = Delay;
//pub use crate::alt_delay::{AltDelay as DelayType};



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    timer::Delay,
    //timer::SysDelay as Delay,
    pac::TIM2,
};

#[cfg(feature = "stm32f1xx")]
pub type DelayType = Delay<TIM2, 1000000_u32>;



//#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
//use stm32f3xx_hal::{
//    delay::Delay,
//    pac::TIM2,
//};

#[cfg(feature = "stm32f3xx")]
pub use crate::alt_delay::{AltDelay as DelayType};
//pub type DelayType = Delay;



#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    timer::Delay,
    //timer::SysDelay as Delay,
    pac::TIM2,
};

#[cfg(feature = "stm32f4xx")]
pub type DelayType = Delay<TIM2, 1000000_u32>;



//#[cfg(feature = "stm32f7xx")]
//use stm32f7xx_hal::{
//    delay::Delay,
//    pac::TIM2,
//};

#[cfg(feature = "stm32f7xx")]
pub use crate::alt_delay::{AltDelay as DelayType};
//pub type DelayType = Delay;  //<TIM2, 1000000_u32>;



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    timer::delay::Delay,
    pac::TIM2,
};

#[cfg(feature = "stm32g0xx")]
pub type DelayType = Delay<TIM2>;



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    delay::Delay,
    //timer::SysDelay as Delay,
    stm32::TIM2,
};

#[cfg(feature = "stm32g4xx")]
pub type DelayType = Delay;



//#[cfg(feature = "stm32h7xx")]
//use stm32h7xx_hal::{
//    delay::Delay,
//    pac::TIM2,
//};

#[cfg(feature = "stm32h7xx")]
pub use crate::alt_delay::{AltDelay as DelayType};
//pub type DelayType = Delay; //<TIM2, 1000000_u32>;



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
};

#[cfg(feature = "stm32l0xx")]
pub type DelayType = Delay<>;
//pub use crate::alt_delay::{AltDelay as DelayType};



//#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
//use stm32l1xx_hal::{
//    delay::Delay,
//    stm32::TIM2,
//};

#[cfg(feature = "stm32l1xx")]
pub use crate::alt_delay::{AltDelay as DelayType};
//pub type DelayType = Delay; //<TIM2, 1000000_u32>;



//#[cfg(feature = "stm32l4xx")]
//use stm32l4xx_hal::{
//    delay::Delay,
//    pac::TIM2,
//};

#[cfg(feature = "stm32l4xx")]
pub use crate::alt_delay::{AltDelay as DelayType};
//pub type DelayType = Delay; //<TIM2, 1000000_u32>;
