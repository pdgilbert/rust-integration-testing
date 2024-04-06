
pub use stm32h7xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};


pub use stm32h7xx_hal::{
      rcc::CoreClocks as Clocks,
      delay::DelayFromCountDownTimer,
      gpio::{gpioa::PA8},
};

//   //////////////////////////////////////////////////////////////////////

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
pub type TxType = Tx<USART1>;

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
   let i2cx = ccdr.peripheral.I2C1;  //.I2C4;

   let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
   let mut led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
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


   (pin, i2c, led, tx, delay, clocks)
}

