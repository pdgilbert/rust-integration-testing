//! Note that (on board) led pin settings are specific to a board used for testing, despite the cfg feature flags suggesting it may be for a HAL.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal;

// "hal" is used for items that are the same in all hal  crates
use crate::stm32xxx_as_hal::hal;

//#[cfg(not(feature = "stm32f4xx"))] //if giving warning with stm32f4xx but IS NEEDED for stm32h7xx to resolve some traits.
use hal::{
    pac::Peripherals,
    gpio::{Output, PushPull},
    prelude::*,
};

// impl LED would work in function signature but does not work in rtic share
// or implimentation of methods,  so LedType is defined:

pub type LedType = LEDPIN<Output<PushPull>>;

pub trait LED: OutputPin {  // see The Rust Programming Language, section 19, Using Supertraits...
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // A default of set_low() for on is defined here, but implementation should deal with a difference
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }

    // Note these default methods use delay so DO NOT USE IN rtic.

    fn blink(&mut self, time: u16, delay: &mut impl DelayNs) -> () {
        self.on();
        delay.delay_ms(time.into());
        self.off();
        delay.delay_ms(time.into()); //consider delay.delay_ms(500);
    }

    fn blink_ok(&mut self, delay: &mut impl DelayNs) -> () {
        let dot: u16 = 5;
        let dash: u16 = 200;
        let spc: u16 = 500;
        let space: u16 = 1000;
        let end: u16 = 1500;

        // dash-dash-dash
        self.blink(dash.into(), delay);
        delay.delay_ms(spc.into());
        self.blink(dash.into(), delay);
        delay.delay_ms(spc.into());
        self.blink(dash.into(), delay);
        delay.delay_ms(space.into());

        // dash-dot-dash
        self.blink(dash.into(), delay);
        delay.delay_ms(spc.into());
        self.blink(dot.into(), delay);
        delay.delay_ms(spc.into());
        self.blink(dash.into(), delay);
        delay.delay_ms(end.into());
    }
}




#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}},};

#[cfg(feature = "stm32f0xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = cortex_m::interrupt::free(move |cs| gpiox.pc13.into_push_pull_output(cs));
    impl LED for LedType {}    
    led
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}},};

#[cfg(feature = "stm32f1xx")]
pub fn setup_led(mut gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output(&mut gpiox.crh);
    impl LED for LedType {} 
    led
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{gpio::{gpioe::{PE15 as LEDPIN, Parts}},};

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



#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}},};


#[cfg(feature = "stm32f4xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();
    impl LED for LedType {}    
    led
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{gpio::{gpioc::{PC13 as LEDPIN,  Parts}}};

#[cfg(feature = "stm32f7xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();
    impl LED for LedType {}
    led
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}}}; //NOT SURE WHAT PIN THIS SHOULD BE

#[cfg(feature = "stm32g0xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();
    impl LED for LedType {}   
    led
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}}}; //NOT SURE WHAT THIS SHOULD BE

#[cfg(feature = "stm32g4xx")]
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



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}}};

#[cfg(feature = "stm32h7xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output();
    impl LED for LedType {}
    led
}


#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}}};

#[cfg(feature = "stm32l0xx")]
pub fn setup_led(gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output(); 
    impl LED for LedType {}
    led
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{gpio::{gpioc::PC9 as LEDPIN,  Input, Floating}};

#[cfg(feature = "stm32l1xx")]
pub fn setup_led(pin: PC9<Input<Floating>>) -> LedType {
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
use stm32l4xx_hal::{gpio::{gpioc::{PC13 as LEDPIN, Parts}}};

#[cfg(feature = "stm32l4xx")]
pub fn setup_led(mut gpiox: Parts) -> LedType {
    let led = gpiox.pc13.into_push_pull_output(&mut gpiox.moder, &mut gpiox.otyper); 
    impl LED for LedType {}
    led
}



//  following are wrapper to do setup_led_from_dp(dp: Peripherals) -> LedType

#[cfg(feature = "stm32f0xx")]
pub fn setup_led_from_dp(mut dp: Peripherals) -> LedType {    
   let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
   setup_led(dp.GPIOC.split(&mut rcc))
}


#[cfg(feature = "stm32f1xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   setup_led(dp.GPIOC.split())
}


#[cfg(feature = "stm32f3xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   let mut rcc = dp.RCC.constrain();
   setup_led(dp.GPIOE.split(&mut rcc.ahb))
}


#[cfg(feature = "stm32f4xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   setup_led(dp.GPIOC.split())
}


#[cfg(feature = "stm32f7xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   setup_led(dp.GPIOC.split())
}


#[cfg(feature = "stm32g0xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   let mut rcc = dp.RCC.constrain();
   setup_led(dp.GPIOC.split(&mut rcc))
}



#[cfg(feature = "stm32g4xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
    let mut rcc = dp.RCC.constrain();
    setup_led(dp.GPIOC.split(&mut rcc))
}

// or

#[cfg(feature = "stm32g4xxXX")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
    let mut rcc = dp.RCC.constrain();
    let gpioc = dp.GPIOC.split(&mut rcc);

    let mut led: LedType = gpioc.pc13.into_push_pull_output();
    led.off();

    led
}


#[cfg(feature = "stm32h7xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC))
}


#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
  let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
  setup_led(dp.GPIOC.split(&mut rcc))
}


#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l1xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
   setup_led(dp.GPIOC.split(&mut rcc).pc9)
}


#[cfg(feature = "stm32l4xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
   let mut rcc = dp.RCC.constrain();
   setup_led(dp.GPIOC.split(&mut rcc.ahb2))
}
