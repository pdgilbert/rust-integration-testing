pub use embedded_hal::delay::DelayNs;

// TRY TO CLEANUP OR REMOVE SOME TYPES IF EH-1.0  AND impl Delay HELP


#[cfg(feature = "stm32f0xx")]
pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay;

#[cfg(feature = "stm32f0xx")]
pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay;


#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    timer::Delay,
    //timer::SysDelay as Delay,
    pac::{TIM2, TIM3}
};

#[cfg(feature = "stm32f1xx")]
pub type Delay1Type = Delay<TIM2, 1000000_u32>;

#[cfg(feature = "stm32f1xx")]
pub type Delay2Type = Delay<TIM3, 1000000_u32>;



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    pac::{TIM2, TIM3}
};

#[cfg(feature = "stm32f3xx")]
pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay;

#[cfg(feature = "stm32f3xx")]
pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay;



#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    pac::{TIM2, TIM5},
    timer::Delay, 
};

#[cfg(feature = "stm32f4xx")]
pub type Delay1Type = Delay<TIM2, 1000000_u32>;

#[cfg(feature = "stm32f4xx")]
pub type Delay2Type = Delay<TIM5, 1000000_u32>;



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    timer::Delay,
    pac::{TIM2, TIM5}
};

#[cfg(feature = "stm32f7xx")]
pub type Delay1Type = Delay<TIM2, 1000000_u32>;

#[cfg(feature = "stm32f7xx")]
pub type Delay2Type = Delay<TIM5, 1000000_u32>;



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    timer::delay::Delay,
    pac::{TIM2, TIM3}
};

#[cfg(feature = "stm32g0xx")]
pub type Delay1Type = Delay<TIM2>;

#[cfg(feature = "stm32g0xx")]
pub type Delay2Type = Delay<TIM3>;



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    timer::CountDownTimer,
    delay::DelayFromCountDownTimer,
    //delay::Delay,
    //timer::SysDelay as Delay,
    pac::{TIM2, TIM3}, 
};

#[cfg(feature = "stm32g4xx")]
pub type Delay1Type = DelayFromCountDownTimer<CountDownTimer<TIM2>>;

#[cfg(feature = "stm32g4xx")]
pub type Delay2Type = DelayFromCountDownTimer<CountDownTimer<TIM3>>;



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
   timer::Timer,
   //delay::Delay,
   delay::DelayFromCountDownTimer,
   pac::{TIM2, TIM5},
};

#[cfg(feature = "stm32h7xx")]
pub type Delay1Type = DelayFromCountDownTimer<TIM2>;
//pub type Delay1Type = DelayFromCountDownTimer<CountDown<TIM2>>;
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;

#[cfg(feature = "stm32h7xx")]
pub type Delay2Type = DelayFromCountDownTimer<Timer<TIM5>>;
//pub type Delay2Type = DelayFromCountDownTimer<TIM5>;
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM5, 1000000_u32>;
//pub type Delay2Type = Delay;
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub type Delay2Type = Delay<TIM5, 1000000_u32>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM5, 1000000_u32>;



//#[cfg(feature = "stm32l0xx")]
//use stm32l0xx_hal::{
//    delay::Delay,
//};

#[cfg(feature = "stm32l0xx")]
pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay<>;


#[cfg(feature = "stm32l0xx")]
pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay<>;


//#[cfg(feature = "stm32l1xx")] 
//use stm32l1xx_hal::{
//    delay::Delay,
//    stm32::{TIM2, TIM5}
//};

#[cfg(feature = "stm32l1xx")]
pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;

#[cfg(feature = "stm32l1xx")]
pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM5, 1000000_u32>;



//#[cfg(feature = "stm32l4xx")]
//use stm32l4xx_hal::{
//    delay::Delay,
//    pac::{TIM2, TIM3}
//};

#[cfg(feature = "stm32l4xx")]
pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;

#[cfg(feature = "stm32l4xx")]
pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM3, 1000000_u32>;
