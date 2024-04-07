
pub use stm32l0xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{gpioa::PA8, Output, OpenDrain},
      prelude::*,
};


use stm32l0xx_hal::{
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Serial1Ext, },
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type TxType = Tx<USART1>;

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
   //let clocks = rcc.clocks;

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpioa = dp.GPIOA.split(&mut rcc);

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c1 = i2c1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    let scl = gpiob.pb10.into_open_drain_output(); 
    let sda = gpiob.pb11.into_open_drain_output();
    let i2c2 = i2c2.i2c(sda, scl, 400_000.Hz(), &mut rcc);
   let (tx, _rx) = dp.USART1.usart(
        gpioa.pa9,
        gpioa.pa10,
        Config::default().baudrate(115200.Bd()),
        &mut rcc,
    )
    .unwrap()
    .split();

    let led = gpiox.pc13.into_push_pull_output(); 
   led.off();

   let delay = DelayType{};
   //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

