pub use stm32l1xx_hal as hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

use stm32l1xx_hal::{  // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
      rcc::Config as rccConfig,
      serial::{Config, SerialExt, },
      //serial::{Config, Rx, Serial1Ext, Serial2Ext, Serial4Ext, Tx},
      gpio::{gpioa::PA8},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {  // not default
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

pub type TxType = Tx<USART1>;
pub type RxType = Rx<USART1>;

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

//use embedded_hal::digital::v2::OutputPin;

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, RxType, SpiType, SpiExt, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rccConfig::hsi());

   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);
   let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
   let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
   let i2c1 = i2c1.i2c((scl, sda), 400.khz(), &mut rcc);

   let scl = gpiob.pb10.into_open_drain_output();
   let sda = gpiob.pb11.into_open_drain_output();
   let i2c2 = i2c2.i2c((scl, sda), 400.khz(), &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc).pc9);
   led.off();

   let spi1 = Spi::new(
       dp.SPI1,
       (
           gpioa.pa5.into_alternate(), // sck  
           gpioa.pa6.into_alternate(), // miso 
           gpioa.pa7.into_alternate(), // mosi 
       ),
       MODE,
       8.MHz(),
       &clocks,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa1.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(), //ResetPin   
        };   

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


   (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}

