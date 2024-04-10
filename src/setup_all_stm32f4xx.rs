use stm32f4xx_hal as hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

// above are commom to all hals. Below are possibly different.

pub use stm32f4xx_hal::{
    pac::{TIM5},
    rcc::{Clocks, RccExt},
    timer::TimerExt,
    serial::{config::Config},
    gpio::{GpioExt, Pin}, 
    gpio::{gpioa::PA8},
    gpio::{gpioc::{PC13 as LEDPIN}},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type; 

// impl LED would work in function signature but does not yet work in rtic share
// or implimentation of methods,  so LedType is defined:
pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type Tx1Type = Tx<USART1>;
pub type Rx1Type = Rx<USART1>;
pub type TxType = Tx1Type;
pub type RxType = Rx1Type;

pub type Tx2Type = Tx<USART2>;
pub type Rx2Type = Rx<USART2>;

pub type SpiType =  Spi<SPI1>;
pub struct SpiExt { pub cs:    Pin<'A', 1, Output>, 
                    pub busy:  Pin<'B', 4>, 
                    pub ready: Pin<'B', 5>, 
                    pub reset: Pin<'A', 0, Output>
}


// this really should be set in example code
pub const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, RxType, SpiType, SpiExt, Delay, Clocks) {
   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   
   let gpioa = dp.GPIOA.split();
   let gpiob = dp.GPIOB.split();
   let gpioc   = dp.GPIOC.split();

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.


   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let sda = gpiob.pb9.into_alternate_open_drain(); 
   let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

   let scl = gpiob.pb10.into_alternate_open_drain();
   let sda = gpiob.pb3.into_alternate_open_drain();
   let i2c2 = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

   let mut led = gpioc.pc13.into_push_pull_output();
   led.off();

   let spi1 = Spi::new(
       dp.SPI1,
       (
           gpioa.pa5.into_alternate(), // sck  
           gpioa.pa6.into_alternate(), // miso 
           gpioa.pa7.into_alternate(), // mosi 
       ),
       MODE, 8.MHz(), &clocks,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa1.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(), //ResetPin   
        };   

   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   let (tx, rx) = Serial::new(dp.USART1, (tx, rx), Config::default().baudrate(115200.bps()), &clocks,
                      ).unwrap().split();

   let delay = dp.TIM5.delay(&clocks);

   (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}


