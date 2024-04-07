use stm32f4xx_hal as hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1, I2C1, I2C2},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

// above are commom to all hals. Below are different.

pub use stm32f4xx_hal::{
    rcc::{Clocks, RccExt},
    timer::TimerExt,
    gpio::GpioExt,
    gpio::{gpioa::PA8},
    serial::{config::Config},
    gpio::{gpioc::{PC13 as LEDPIN}},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type; 

// NEXT SHOULD BE HERE BUT NEEDED IN  src/i2c1_i2c2_led_delay.
// WHICH uses crate::led::{setup_led, LED, LedType};

// impl LED would work in function signature but does not work in rtic share
// or implimentation of methods,  so LedType is defined:
pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type TxType = Tx<USART1>;

//   //////////////////////////////////////////////////////////////////////



pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let gpiob = dp.GPIOB.split();

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);
   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let sda = gpiob.pb9.into_alternate_open_drain(); 
   let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

   let scl = gpiob.pb10.into_alternate_open_drain();
   let sda = gpiob.pb3.into_alternate_open_drain();
   let i2c2 = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

   //let mut led = setup_led(dp.GPIOC.split()); 
   let gpioc   = dp.GPIOC.split();
   let mut led = gpioc.pc13.into_push_pull_output();
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

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}


