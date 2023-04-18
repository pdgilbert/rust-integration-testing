//  compare with dht_i2c_led_delay.rs

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use crate::dp::{Peripherals};
pub use crate::delay::{Delay1Type, Delay2Type};
pub use crate::led::{setup_led, LED, LedType};

//#[cfg(not(feature = "stm32f0xx"))]
//pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType};
//
//#[cfg(feature = "stm32f0xx")]
//pub use crate::i2c::{I2c2Type as I2cType};  // TO RESOLVE stm32f0xx CONFLICT

pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType};
pub use crate::i2c::{setup_i2c1_i2c2, setup_i2c2, I2c1Type, I2c2Type};

pub use embedded_hal::blocking::delay::DelayUs;
pub use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs as DelayMs;

// A delay is used in sensor initialization and read. 
// Systick is used by monotonic (for spawn), so delay needs to use a timer other than Systick
// asm::delay used in AltDelay is not an accurate timer but gives a delay at least 
//  number of indicated clock cycles.


pub fn setup_i2c1_i2c2_led_delays() ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {    
   //let cp = CorePeripherals::take().unwrap();
   let dp = Peripherals::take().unwrap();
   let (i2c1, i2c2, led, delay1, delay2) = setup_i2c1_i2c2_led_delays_using_dp(dp);
   (i2c1, i2c2, led, delay1, delay2)
}


pub fn setup_i2c1_i2c2_led_delay() ->  (I2cType, I2c2Type, LedType, Delay1Type) {    
   //let cp = CorePeripherals::take().unwrap();
   let dp = Peripherals::take().unwrap();
   let (i2c1, i2c2, led, delay1, _delay2) = setup_i2c1_i2c2_led_delays_using_dp(dp);
   (i2c1, i2c2, led, delay1)
}

pub fn setup_i2c1_i2c2_led_delay_using_dp(dp: Peripherals) -> (
             I2cType, I2c2Type, LedType, Delay1Type) {
   let (i2c1, i2c2, led, delay1, _delay2) = setup_i2c1_i2c2_led_delays_using_dp(dp);  

   (i2c1, i2c2, led, delay1)
}

#[cfg(feature = "stm32f0xx")]
use stm32f0xx_hal::{
    //delay::Delay,
    //pac::{CorePeripherals},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32f0xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(mut dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {    
   let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
   let gpiob = dp.GPIOB.split(&mut rcc);

   let (i2c1, i2c2)  = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob,  &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();
   
   //let cp = CorePeripherals::take().unwrap();
   //let delay1 = Delay::new(cp.SYST, &rcc);
   let delay1 = Delay1Type{};
   let delay2 = Delay2Type{};
   //let delay1 = dp.TIM1.delay_ms(&rcc);
   //let delay2 = dp.TIM3.delay_ms(&rcc);

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{prelude::*};

#[cfg(feature = "stm32f1xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32f1xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   let rcc = dp.RCC.constrain();
   let mut afio = dp.AFIO.constrain();
   let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

   let gpiob = dp.GPIOB.split();

   //afio  needed for i2c1 (PB8, PB9) but not i2c2
   // Next does not work because of value move problems. Using &mut argument and * deref only gets to
   // cannot move out of `gpiob.pb8` which is behind a mutable reference.
   // A better solution to this is need - maybe a config or into method for i2c1 and i2c2 ?
   //let i2c1 = setup_i2c1(dp.I2C1, gpiob, &mut afio, &clocks);
   //let i2c2 = setup_i2c2(dp.I2C2, gpiob, &clocks);
   // As work-around combine in setup_i2c1_i2c2.

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &mut afio, &clocks);
   
   let mut led = setup_led(dp.GPIOC.split()); 
   led.off();

   let delay1 = dp.TIM2.delay_us(&clocks);
   let delay2 = dp.TIM3.delay_us(&clocks);

   (i2c1, i2c2, led, delay1, delay2)
   }



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{prelude::*,};

#[cfg(feature = "stm32f3xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32f3xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

   let gpioa = dp.GPIOA.split(&mut rcc.ahb);

   // setup_i2c2 does not work. There is a "value used here after partial move" problem with I2C2 on gpioa
   //    because gpioa is used above. And the only option for I2C2 seems to be gpioa.
   //    gpioa can be used without setup_i2c2 using
   // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
   // let scl =  gpioa.pa9.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
   // let sda = gpioa.pa10.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
   //    // //NOT sure if pull up is needed
   //    scl.internal_pull_up(&mut gpiob.pupdr, true);
   //    sda.internal_pull_up(&mut gpiob.pupdr, true);
   // let i2c1 = I2c::new(dp.I2C2, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);

   let gpiob = dp.GPIOB.split(&mut rcc.ahb);
   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpioa, gpiob, clocks, rcc.apb1);

   let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
   led.off();

   let delay1 = Delay1Type{};
   let delay2 = Delay2Type{};
   //delay(clocks.sysclk().0 / 100);  something like this??
   //let delay1 = dp.TIM2.delay_ms(&clocks);
   //let delay2 = dp.TIM3.delay_ms(&clocks);

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{prelude::*,};

#[cfg(feature = "stm32f4xx")]
pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32f4xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   let gpiob = dp.GPIOB.split();

   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &clocks);

   let mut led = setup_led(dp.GPIOC.split()); 
   led.off();

   let delay1 = dp.TIM2.delay_us(&clocks);
   let delay2 = dp.TIM5.delay_us(&clocks);

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{prelude::*};

#[cfg(feature = "stm32f7xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32f7xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   let gpiob = dp.GPIOB.split();

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &clocks, &mut rcc.apb1);

   let led = setup_led(dp.GPIOC.split());
   //let delay = DelayType{};
   let delay1 = dp.TIM2.delay_us(&clocks);
   let delay2 = dp.TIM5.delay_us(&clocks);

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{prelude::* };

#[cfg(feature = "stm32g0xx")]
pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32g0xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {

   let mut rcc = dp.RCC.constrain();
   let gpiob = dp.GPIOB.split(&mut rcc);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &mut rcc);
   
   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let delay1 = dp.TIM2.delay(&mut rcc);
   let delay2 = dp.TIM3.delay(&mut rcc);

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    timer::Timer,
    delay::DelayFromCountDownTimer,
    prelude::*,
};

#[cfg(feature = "stm32g4xx")]
pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32g4xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {

   let mut rcc = dp.RCC.constrain();

   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpioa, gpiob, &mut rcc);

   let mut led = setup_led(gpioc); 
   led.off();

   let timer1 = Timer::new(dp.TIM2, &rcc.clocks);
   let delay1 = DelayFromCountDownTimer::new(timer1.start_count_down(100.ms()));

   let timer2 = Timer::new(dp.TIM3, &rcc.clocks);
   let delay2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.ms()));

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{prelude::*,};

#[cfg(feature = "stm32h7xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32h7xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
   let i2cx1 = ccdr.peripheral.I2C1;

   let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
   let i2cx2 = ccdr.peripheral.I2C2;

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, gpiob, i2cx1,  dp.I2C2, gpiof, i2cx2, &clocks);

   let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
   let delay1 = Delay1Type{};
   let delay2 = Delay2Type{};
   //let delay1 = dp.TIM2.delay_ms(&mut rcc);
   //let delay2 = dp.TIM5.delay_ms(&mut rcc);
   // try building method like 
   //let mut timer = dp.TIM2.timer(1.Hz(), ccdr.peripheral.TIM2, &ccdr.clocks);
   // 20ms wait with timer
   // timer.start(MilliSeconds::from_ticks(20).into_rate());
   // block!(timer.wait()).ok();

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
   //delay::Delay,
   prelude::*,
    rcc::Config, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32l0xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   // UNTESTED
   let mut rcc = dp.RCC.freeze(Config::hsi16());
   //let clocks = rcc.clocks;
   let gpiob = dp.GPIOB.split(&mut rcc);

   let led = setup_led(dp.GPIOC.split(&mut rcc));
   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, rcc);

   let delay1 = Delay1Type{};
   let delay2 = Delay2Type{};
   //let delay1 = dp.TIM2.delay(&mut rcc);
   //let delay2 = dp.TIM3.delay(&mut rcc);

   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    rcc, // for ::Config but note name conflict with serial
    prelude::*,
};

#[cfg(feature = "stm32l1xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32l1xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
   let gpiob = dp.GPIOB.split(&mut rcc);

   let led = setup_led(dp.GPIOC.split(&mut rcc).pc9);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, rcc);
 
   let delay1 = Delay1Type{};
   let delay2 = Delay2Type{};
   //let delay1 = dp.TIM2.delay_ms(&mut rcc);
   //let delay2 = dp.TIM5.delay_ms(&mut rcc);
 
   (i2c1, i2c2, led, delay1, delay2)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{prelude::*,};

#[cfg(feature = "stm32l4xx")]
pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

#[cfg(feature = "stm32l4xx")]
pub fn setup_i2c1_i2c2_led_delays_using_dp(dp: Peripherals) ->  (I2cType, I2c2Type, LedType, Delay1Type, Delay2Type) {
   let mut flash = dp.FLASH.constrain();
   let mut rcc = dp.RCC.constrain();
   let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
   let clocks = rcc.cfgr.sysclk(80.MHz()).pclk1(80.MHz()).pclk2(80.MHz()).freeze(&mut flash.acr, &mut pwr);

   let gpiob = dp.GPIOB.split(&mut rcc.ahb2);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2,  gpiob, &clocks, &mut rcc.apb1r1);

   let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
   let delay1 = Delay1Type{};
   let delay2 = Delay2Type{};
   //let delay1 = dp.TIM2.delay_ms(&clocks);
   //let delay2 = dp.TIM3.delay_ms(&clocks);

   (i2c1, i2c2, led, delay1, delay2)
}


