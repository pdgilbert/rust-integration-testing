
pub use stm32g4xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

// above are commom to all hals. Below are different.

pub use stm32g4xx_hal::{
    rcc::{Clocks},
    time::ExtU32,
    timer::Timer,
    delay::DelayFromCountDownTimer,
    gpio::{ Alternate, gpioa::{PA8, PA9} },
    serial::{FullConfig, NoDMA},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type TxType = Tx<USART1, PA9<Alternate<7_u8>>, NoDMA >;
//pub type TxType = Tx<USART1, PA9<Output<PushPull>>, NoDMA >;


pub type OpenDrainType = PA8<Output<OpenDrain>>;


//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().unwrap(); // Pull high to avoid confusing the sensor when initializing.

   let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let clocks = rcc.clocks;  // not sure if this is right
   
   let timer2 = Timer::new(dp.TIM3, &clocks);
   let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));
   //let timer2 = Timer::new(dp.TIM2, &clocks);
   //let mut delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));


   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   //let (tx, _rx) = Serial::new(dp.USART1,(tx, rx),...  would be nice
   let (tx, _rx) = dp.USART1.usart(tx, rx, FullConfig::default().baudrate(115200.bps()),
         &mut rcc).unwrap().split();

   (pin, i2c, led, tx, delay, clocks)
}

