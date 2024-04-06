
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32f3xx_hal::{
    gpio::{gpioa::{PA8, PA9}, PushPull, AF7 },
};

//   //////////////////////////////////////////////////////////////////////


pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
pub type TxType = Tx<USART1, PA9<AF7<PushPull>>>;
//pub type TxType = Tx<USART1, impl TxPin<USART1>>;  // impl is unstable in type alias
// See  https://github.com/stm32-rs/stm32f3xx-hal/issues/288
//   regarding why it is necessary to specify the concrete pin here.


//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) -> (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut flash = dp.FLASH.constrain();
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze(&mut flash.acr);

   let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
   let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc.ahb);
   let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);

   let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
   led.off();

   let delay = DelayType{};

   let (tx, _rx) = Serial::new(
       dp.USART1,
       (
           gpioa
               .pa9
               .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
           gpioa
               .pa10
               .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
       ),
       115200.Bd(),
       clocks,
       &mut rcc.apb2,
   )
   .split();

   (pin, i2c, led, tx, delay, clocks)
}

