
    #[cfg(feature = "stm32f0xx")]
    pub use stm32f0xx_hal::pac::{Peripherals};

    #[cfg(feature = "stm32f1xx")]
    pub use stm32f1xx_hal::pac::{Peripherals};
    
    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    pub use stm32f3xx_hal::pac::{Peripherals};
    
    #[cfg(feature = "stm32f4xx")]
    pub use stm32f4xx_hal::pac::{Peripherals};
    
    #[cfg(feature = "stm32f7xx")]
    pub use stm32f7xx_hal::pac::{Peripherals};
     
    #[cfg(feature = "stm32g4xx")]
    pub use stm32g4xx_hal::pac::{Peripherals};
      
    #[cfg(feature = "stm32g0xx")]
    pub use stm32g0xx_hal::pac::{Peripherals};
  
    #[cfg(feature = "stm32h7xx")]
    pub use stm32h7xx_hal::pac::{Peripherals};
    
    #[cfg(feature = "stm32l0xx")]
    pub use stm32l0xx_hal::pac::{Peripherals};
        
    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    pub use stm32l1xx_hal::stm32::{Peripherals};
    
    #[cfg(feature = "stm32l4xx")]
    pub use stm32l4xx_hal::pac::{Peripherals};
    
