pub use embedded_hal::delay::DelayNs;

#[cfg(feature = "stm32f0xx")] //  eg  stm32f303x4
use stm32f0xx_hal::delay::Delay;

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::timer::SysDelay as Delay;

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::delay::Delay;

//#[cfg(feature = "stm32f4xx")] 
//use stm32f4xx_hal::timer::SysDelay as Delay;

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::timer::SysDelay as Delay;

#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::timer::delay::Delay;


#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    timer::{Timer, CountDownTimer},
    delay::DelayFromCountDownTimer,
    stm32::{TIM2, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32g4xx")]
pub type DelayType = DelayFromCountDownTimer<CountDownTimer<TIM2>>;


#[cfg(feature = "stm32h7xx")]
pub use stm32h7xx_hal::delay::Delay;

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::delay::Delay;

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::delay::Delay;

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::delay::Delay;

