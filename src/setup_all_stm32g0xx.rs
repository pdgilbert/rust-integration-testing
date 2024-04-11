pub use crate::stm32g0xx_hal as hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      timer::{Delay as halDelay},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

pub use stm32g0xx_hal::{
    pac::{TIM2, TIM3}
    serial::{FullConfig},
    gpio::{gpioa::PA8},
    gpio::{gpioc::{PC13 as LEDPIN}},
};

//   /
pub use embedded_hal::delay::DelayNs;
/////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

pub type Delay1Type = halDelay<TIM2>;
pub type Delay2Type = halDelay<TIM3>;
pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

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

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, RxType, SpiType, SpiExt, Delay, Clocks) {
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

   let tx = gpioa.pa9; 
   let rx = gpioa.pa10;
   let (tx, _rx) = dp.USART1.usart((tx, rx), FullConfig::default(), &mut rcc).unwrap().split();

   let delay = dp.TIM2.delay(&mut rcc);

   (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}

