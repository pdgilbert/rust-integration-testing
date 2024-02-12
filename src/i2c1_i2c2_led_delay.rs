// In addition to i2c1, i2c2, and led, this returns a non-systick delay as well as clocks.
// This allows for use in both rtic and non-rtic code. 
// Since cp is not an argument, a cp.SYST delay cannot be returned, but the returned clocks
// can be used to get a systick delay:  Delay::new(cp.SYST, clocks).
// Returning the delay is useful in rtic code because it avoids the
// problem caused by i2c1_i2c2_led::setup(cx.device) moving cx.device,
// and then not being available for cx.device.TIM2.delay(&clocks).
//
// A delay is used in some sensor initializations and read operations. Often the sensor crate 
// requires taking ownership. But Systick is used by monotonic (for spawn), so any delay in 
// rtic code needs to use a timer other than Systick. For  this purpose the returned delay is useful.

//  Usage    CLEAN UP
//    let cp = CorePeripherals::take().unwrap();
//    let dp = Peripherals::take().unwrap();
//    let (i2c1, i2c2, mut led, mut delay, clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);
//
//    let mut delay1 = Delay::new(cp.SYST, clocks); 
//    or
//    let delay1 = dp.TIM2.delay(&clocks);
//    let delay2 = dp.TIM5.delay(&clocks);
//    or
//    let delay1 = dp.TIM2.delay_ms(&clocks);
//    let delay2 = dp.TIM3.delay_ms(&clocks);
//    or
//    let delay1 = dp.TIM2.delay_us(&clocks);
//    let delay2 = dp.TIM5.delay_us(&clocks);
//    or
//    let delay1 = dp.TIM2.delay_ns(&clocks);
//    let delay2 = dp.TIM5.delay_ns(&clocks);
//    or ?
//    let delay = Delay{};
//    


#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::monoclock::{MONOCLOCK};

use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1_i2c2, I2c1Type, I2c2Type};

use crate::delay::{Delay2Type as Delay};

// "hal" is used for items that are the same in all hal  crates
use crate::stm32xxx_as_hal::hal;

use hal::{
   pac::{Peripherals},
   gpio::GpioExt,
};
#[cfg(not(feature = "stm32f4xx"))]
use hal::{
   prelude::*,        // this gives unused warning with stm32f4xx but IS NEEDED for stm32h7xx to resolve some traits.         
};



#[cfg(feature = "stm32f0xx")]
use stm32f0xx_hal::{
   //delay::Delay,
   //pac::{CorePeripherals},
   prelude::*,
};

#[cfg(feature = "stm32f0xx")]
pub fn setup_from_dp(mut dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {    
   let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
   let gpiob = dp.GPIOB.split(&mut rcc);

   let (i2c1, i2c2)  = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob,  &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();
   
   let delay = Delay{};
   //let delay = dp.TIM1.delay_ms(&rcc);

   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{rcc::Clocks};

#[cfg(feature = "stm32f1xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
   let rcc = dp.RCC.constrain();
   let mut afio = dp.AFIO.constrain();
   let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

   let gpiob = dp.GPIOB.split();

   //afio  needed for i2c1 (PB8, PB9) but not i2c2

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &mut afio, &clocks);
   
   let mut led = setup_led(dp.GPIOC.split()); 
   led.off();

   let delay = dp.TIM2.delay_us(&clocks);

   (i2c1, i2c2, led, delay, clocks)
   }



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{prelude::*,};

#[cfg(feature = "stm32f3xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
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

   let delay = Delay{};
   
   //let delay = dp.TIM2.delay_ms(&clocks);

   (i2c1, i2c2, led, delay, clocks)
}




//#[cfg(feature = "stm32f4xx")]
//use stm32f4xx_hal::{prelude::*,};

#[cfg(feature = "stm32f4xx")]
pub use stm32f4xx_hal::{
   rcc::{Clocks, RccExt},
   pac::{TIM2, TIM5},
   //timer::Delay,
   timer::TimerExt,
};


#[cfg(feature = "stm32f4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
   let gpiob = dp.GPIOB.split();

   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &clocks);

   let mut led = setup_led(dp.GPIOC.split()); 
   led.off();

   let delay = dp.TIM5.delay(&clocks);

   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{prelude::*};

#[cfg(feature = "stm32f7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   let gpiob = dp.GPIOB.split();

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &clocks, &mut rcc.apb1);

   let led = setup_led(dp.GPIOC.split());
   //let delay = DelayType{};
   let delay1 = dp.TIM2.delay_us(&clocks);
   let delay2 = dp.TIM5.delay_us(&clocks);

   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{prelude::* };

#[cfg(feature = "stm32g0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {

   let mut rcc = dp.RCC.constrain();
   let gpiob = dp.GPIOB.split(&mut rcc);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, &mut rcc);
   
   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let delay = dp.TIM2.delay(&mut rcc);

   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    rcc::{Clocks},
    time::ExtU32,
    timer::Timer,
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32g4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {

   let mut rcc = dp.RCC.constrain();

   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpioa, gpiob, &mut rcc);

   let mut led = setup_led(gpioc); 
   led.off();

   let clocks = rcc.clocks;  // not sure if this is right

   let timer1 = Timer::new(dp.TIM3, &clocks);
   let delay = DelayFromCountDownTimer::new(timer1.start_count_down(100.millis()));

   (i2c1, i2c2, led, delay, clocks)
}



//#[cfg(feature = "stm32h7xx")]
//use stm32h7xx_hal::{
//    pac::{TIM2, TIM5},
//    block,
//    prelude::*,}
//;

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::delay::DelayFromCountDownTimer;

#[cfg(feature = "stm32h7xx")]
pub use stm32h7xx_hal::rcc::CoreClocks as Clocks;

#[cfg(feature = "stm32h7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
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
   // CountDownTimer not supported by embedded-hal 1.0.0 ??
   let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
   //delay::Delay,
   prelude::*,
    rcc::Config, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
   // UNTESTED
   let mut rcc = dp.RCC.freeze(Config::hsi16());
   //let clocks = rcc.clocks;
   let gpiob = dp.GPIOB.split(&mut rcc);

   let led = setup_led(dp.GPIOC.split(&mut rcc));
   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, rcc);

   let delay = Delay1Type{};
   //let delay = dp.TIM2.delay(&mut rcc);

   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    rcc, // for ::Config but note name conflict with serial
    prelude::*,
};

#[cfg(feature = "stm32l1xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
   let gpiob = dp.GPIOB.split(&mut rcc);

   let led = setup_led(dp.GPIOC.split(&mut rcc).pc9);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2, gpiob, rcc);
 
   let delay = Delay1Type{};
   //let delay = dp.TIM2.delay_ms(&mut rcc);
 
   (i2c1, i2c2, led, delay, clocks)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{prelude::*,};

#[cfg(feature = "stm32l4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {
   let mut flash = dp.FLASH.constrain();
   let mut rcc = dp.RCC.constrain();
   let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
   let clocks = rcc.cfgr.sysclk(80.MHz()).pclk1(80.MHz()).pclk2(80.MHz()).freeze(&mut flash.acr, &mut pwr);

   let gpiob = dp.GPIOB.split(&mut rcc.ahb2);

   let (i2c1, i2c2) = setup_i2c1_i2c2(dp.I2C1, dp.I2C2,  gpiob, &clocks, &mut rcc.apb1r1);

   let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
   let delay = Delay1Type{};
   //let delay = dp.TIM2.delay_ms(&clocks);

   (i2c1, i2c2, led, delay, clocks)
}


