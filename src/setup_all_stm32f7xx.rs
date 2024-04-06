
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32f7xx_hal::{
      pac,
      gpio::{gpioa::PA8},
      serial::{Config, Oversampling, DataBits, Parity},
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
   let mut dht   = gpioa .pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

   let mut led = setup_led(dp.GPIOC.split());
   led.off();

   let delay = dp.TIM2.delay_us(&clocks);

   let (tx, _rx) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9   for console
            gpioa.pa10.into_alternate(),
        ), //rx pa10  for console
        &clocks,
        Config {
            baud_rate: 115200.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
   )
   .split();

  (pin, i2c, led, tx, delay, clocks)
}

