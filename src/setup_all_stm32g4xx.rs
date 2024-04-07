
pub use stm32g4xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1, I2C1, I2C2},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

// above are commom to all hals. Below are different.

pub use stm32g4xx_hal::{
    i2c::{Config},// SDAPin, SCLPin},
    rcc::{Clocks},
    time::{ExtU32, RateExtU32},
    timer::Timer,
    delay::DelayFromCountDownTimer,
    gpio::{Alternate, AlternateOD,
           gpioa::{PA8, PA9},
           gpiob::{PB7, PB8, PB9,  PB10, PB11},
           gpioc::{PC4}
    },
    serial::{FullConfig, NoDMA},
    gpio::{gpioc::{PC13 as LEDPIN}},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PB7<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, PB9<AlternateOD<4_u8>>, PB8<AlternateOD<4_u8>>>;
//pub type I2c1Type = I2c<I2C1, impl SCLPin<I2C1>, impl SDAPin<I2C1>>;
pub type I2c2Type = I2c<I2C2, PA8<AlternateOD<4_u8>>, PC4<AlternateOD<4_u8>>>;
//pub type I2c2Type = I2c<I2C2, impl SCLPin<I2C2>, impl SDAPin<I2C2>>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;

impl LED for LedType {  // none default
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

pub type TxType = Tx<USART1, PA9<Alternate<7_u8>>, NoDMA >;
//pub type TxType = Tx<USART1, PA9<Output<PushPull>>, NoDMA >;




//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let mut pin = gpiob.pb7.into_open_drain_output();
   pin.set_high().unwrap(); // Pull high to avoid confusing the sensor when initializing.

   //let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);
   let sda = gpiob.pb9.into_alternate_open_drain(); 
   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let i2c1 = dp.I2C1.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

   let sda = gpioa.pa8.into_alternate_open_drain(); 
   let scl = gpioc.pc4.into_alternate_open_drain();
   let i2c2 = dp.I2C2.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

   let mut led = gpioc.pc13.into_push_pull_output();
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

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

