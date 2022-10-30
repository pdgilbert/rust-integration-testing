#[cfg(feature = "stm32f0xx")]
use stm32f0xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::gpio::{gpioa::PA8, Output, OpenDrain}; 

#[cfg(feature = "stm32f3xx")]
use stm32f3xx_hal::gpio::{gpioa::{PA8,Output, OpenDrain},};

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};

#[cfg(feature = "stm32l1xx")]
use stm32l1xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::gpio::{gpioa::PA8, Output, OpenDrain};


pub type DhtType = PA8<Output<OpenDrain>>;

