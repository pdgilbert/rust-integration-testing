pub use stm32l1xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32l1xx_hal::{  // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
      rcc::Config as rccConfig,
      serial::{Config, SerialExt, },
      //serial::{Config, Rx, Serial1Ext, Serial2Ext, Serial4Ext, Tx},
      gpio::{gpioa::PA8},
};


//   //////////////////////////////////////////////////////////////////////

use embedded_hal::digital::v2::OutputPin;

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
pub type TxType = Tx<USART1>;

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rccConfig::hsi());

   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let mut led = setup_led(dp.GPIOC.split(&mut rcc).pc9);
   led.off();

   let delay = DelayType{};

   let (tx, _rx) = dp
       .USART1
       .usart(
           (gpioa.pa9, gpioa.pa10),
           Config::default().baudrate(115200.bps()),
           &mut rcc,
       )
       .unwrap()
       .split();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);

   (pin, i2c, led, tx, delay, clocks)
}

