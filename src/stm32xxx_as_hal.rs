#[cfg(feature = "stm32f0xx")]
pub use stm32f0xx_hal as hal;

#[cfg(feature = "stm32f1xx")]
pub use stm32f1xx_hal as hal; 

#[cfg(feature = "stm32f3xx")]
pub use stm32f3xx_hal as hal;

#[cfg(feature = "stm32f4xx")]
pub use stm32f4xx_hal as hal;

#[cfg(feature = "stm32f7xx")]
pub use stm32f7xx_hal as hal;

#[cfg(feature = "stm32g0xx")]
pub use stm32g0xx_hal as hal;

#[cfg(feature = "stm32g4xx")]
pub use stm32g4xx_hal as hal;

#[cfg(feature = "stm32h7xx")]
pub use stm32h7xx_hal as hal;

#[cfg(feature = "stm32l0xx")]
pub use stm32l0xx_hal as hal;

#[cfg(feature = "stm32l1xx")]
pub use stm32l1xx_hal as hal;

#[cfg(feature = "stm32l4xx")]
pub use stm32l4xx_hal as hal;

