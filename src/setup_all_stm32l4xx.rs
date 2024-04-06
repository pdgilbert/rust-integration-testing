
pub use stm32l4xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32l4xx_hal::{
      serial::{Config as serialConfig, },
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
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

   let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
   let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
   let mut led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
   led.off();

   let delay = DelayType{};

   let (tx, _rx) = Serial::usart1(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9  for console
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10 for console
        ),
        Config::default().baudrate(115200.bps()),
        clocks,
        &mut rcc.apb2,
   )
   .split();

   (pin, i2c, led, tx, delay, clocks)
}

