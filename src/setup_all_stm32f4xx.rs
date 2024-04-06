use stm32f4xx_hal as hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

// above are commom to all hals. Below are different.

pub use stm32f4xx_hal::{
    rcc::{Clocks, RccExt},
    timer::TimerExt,
    gpio::GpioExt,
    gpio::{gpioa::PA8},
    serial::{config::Config},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
pub type TxType = Tx<USART1>;


//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

   let mut led = setup_led(dp.GPIOC.split()); 
   led.off();

   let delay = dp.TIM5.delay(&clocks);

   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   let (tx, _rx) = Serial::new(
       dp.USART1,
       (tx, rx),
       Config::default().baudrate(115200.bps()),
       &clocks,
   )
   .unwrap()
   .split();

   (pin, i2c, led, tx, delay, clocks)
}


