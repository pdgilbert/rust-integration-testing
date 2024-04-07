
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      pac::{I2C1, I2C2},
      i2c::I2c,
      serial::{Serial, Tx, Error},
      gpio::{gpioa::PA8, Output, OpenDrain},
      prelude::*,
};

pub use stm32g0xx_hal::{
    serial::{FullConfig},
};

//   //////////////////////////////////////////////////////////////////////

pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type TxType = Tx<USART1>;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc);

   //let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);
    let scl = gpiob.pb8.into_open_drain_output_in_state(PinState::High); 
    let sda = gpiob.pb9.into_open_drain_output_in_state(PinState::High); 
    let i2c1 = I2c::i2c1(i2c1,  sda, scl,  i2cConfig::with_timing(0x2020_151b), rcc);

    let scl = gpiob.pb10.into_open_drain_output_in_state(PinState::High);
    let sda = gpiob.pb11.into_open_drain_output_in_state(PinState::High); 
    let i2c2 = I2c::i2c2(i2c2,  sda, scl,  i2cConfig::with_timing(0x2020_151b), rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let tx = gpioa.pa9; 
   let rx = gpioa.pa10;
   let (tx, _rx) = dp.USART1.usart((tx, rx), FullConfig::default(), &mut rcc).unwrap().split();

   let delay = dp.TIM2.delay(&mut rcc);

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

