
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      pac::{I2C1, I2C2},
      i2c::I2c,
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

pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
//pub type I2c1Type = BlockingI2c<I2C1, PB8<Alternate<4u8, OpenDrain>>, PB9<Alternate<4u8, OpenDrain>>>;
//pub type I2c1Type =  BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>;
pub type TxType = Tx<USART1>;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut dht   = gpioa .pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);
    let i2c1 = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

    let scl = gpiob.pb10.into_alternate_open_drain(); 
    let sda = gpiob.pb11.into_alternate_open_drain(); 

    let i2c2 = BlockingI2c::i2c2(
        i2c2,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

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

  (pin, i2c1, i2c2, led, tx, delay, clocks)
}

