//! Note that (on board) led pin settings are specific to a board used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.

pub trait LED {
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // implementation should deal with this difference
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods. Note these use delay so do not use in rtic.
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off();
        delay.delay_ms(time); //consider delay.delay_ms(500u16);
    }
}

// impl LED would work in function signature but does not work in rtic share
// or implimentation of methods,  so LedType is defined in the following.
#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc

use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioc::{PC13, Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f0xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = cortex_m::interrupt::free(move |cs| gpiox.pc13.into_push_pull_output(cs));

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }
    
    led
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{gpioc::{PC13, Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f1xx")]
pub fn setup_led(mut gpiox: Parts) -> LedType {
//pub fn setup_led(mut gpiox: Parts) -> impl LED {
//pub fn setup_led<T>(mut gpiox: T) -> impl LED 
//where T: stm32f1xx_hal::gpio::gpioc::Parts, {
    let led = gpiox.pc13.into_push_pull_output(&mut gpiox.crh);

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    
    led
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioe::{PE15, Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
pub type LedType = PE15<Output<PushPull>>;

#[cfg(feature = "stm32f3xx")]
pub fn setup_led(mut gpiox: Parts) -> LedType {
    let led = gpiox.pe15.into_push_pull_output(&mut gpiox.moder, &mut gpiox.otyper);

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }
    
    led
}


#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{gpioc::{PC13, Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f4xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    
    led
}


#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioc::{PC13,  Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f7xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    
    led
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioc::{PC13, Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32h7xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }
    
    led
}


#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB8, PB9},
        gpioc::{PC13, Parts},
        OpenDrain, Output, PushPull,
    },
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32l0xx")]
pub fn setup_led(mut gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output(); 

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    
    led
}


#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpiob::PB6, Output, PushPull, Input, Floating},
    prelude::*,
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
pub type LedType = PB6<Output<PushPull>>;  // blue user LED on Discovery board

#[cfg(feature = "stm32l1xx")]
pub fn setup_led(pin: PB6<Input<Floating>>) -> LedType {
    //let led = gpiox.pb6.into_push_pull_output(); 
    let led = pin.into_push_pull_output(); 

    impl LED for LedType {
        fn on(&mut self) -> () {
           self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }
    
    led
}


#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioc::{PC13, Parts}, Output, PushPull},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
pub type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32l4xx")]
pub fn setup_led(mut gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output(&mut gpiox.moder, &mut gpiox.otyper); 

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    
    led
}
