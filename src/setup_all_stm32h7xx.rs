
pub use stm32h7xx_hal::{
      pac::{Peripherals, CorePeripherals, I2C1, I2C2, USART1},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};


pub use stm32h7xx_hal::{
      rcc::CoreClocks as Clocks,
      delay::DelayFromCountDownTimer,
      gpio::{gpioa::PA8},
      gpio::{gpioc::PC13 as LEDPIN},
};

//   //////////////////////////////////////////////////////////////////////

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type;
pub type TxType  = Tx<USART1>;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
   let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
   let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
   let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   //let i2cx = ccdr.peripheral.I2C1;  //.I2C4;
   //let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
   let i2c1 =  dp.I2C1.i2c(
                    (gpiob.pb8.into_alternate().set_open_drain(), // scl  
                     gpiob.pb9.into_alternate().set_open_drain(), // sda
                    ), 400.kHz(), ccdr.peripheral.I2C1, &clocks);

   let i2c2 =  dp.I2C2.i2c(
                    (gpiof.pf1.into_alternate().set_open_drain(), // scl
                     gpiof.pf0.into_alternate().set_open_drain(), // sda
                    ), 400.kHz(), ccdr.peripheral.I2C2, &clocks);

   let mut led: LedType = gpioc.pc13.into_push_pull_output();
   led.off();

   // CountDownTimer not supported by embedded-hal 1.0.0 ??
   let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   let (tx, _rx) = dp.USART1.serial(
            (
                gpioa.pa9.into_alternate(), //tx pa9
                gpioa.pa10.into_alternate(),
            ), //rx pa10
            115200.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split();


   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

