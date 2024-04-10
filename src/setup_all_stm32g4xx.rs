use stm32g4xx_hal as hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
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
    spi::{Mode, Phase, Polarity},
    serial::{FullConfig, NoDMA},
    gpio::{Alternate, AlternateOD, Input, Floating,
           gpioa::{PA0, PA1, PA2, PA3, PA5, PA6, PA7, PA8, PA9, PA10},
           gpiob::{PB4, PB5, PB7, PB8, PB9},
           gpioc::{PC4, PC13 as LEDPIN}},
};


//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PB7<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, PB9<AlternateOD<4_u8>>, PB8<AlternateOD<4_u8>>>;
//pub type I2c1Type = I2c<I2C1, impl SCLPin<I2C1>, impl SDAPin<I2C1>>;
pub type I2c2Type = I2c<I2C2, PA8<AlternateOD<4_u8>>, PC4<AlternateOD<4_u8>>>;
//pub type I2c2Type = I2c<I2C2, impl SCLPin<I2C2>, impl SDAPin<I2C2>>;
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

pub type TxType = Tx<USART1, PA9<Alternate<7_u8>>, NoDMA >;
//pub type TxType = Tx<USART1, PA9<Output<PushPull>>, NoDMA >;
pub type RxType = Rx<USART1, PA10<Alternate<7_u8>>, NoDMA >;

pub type SpiType =  Spi<SPI1,(PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;
pub struct SpiExt { pub cs:    PA1<Output<PushPull>>, 
                    pub busy:  PB4<Input<Floating>>, 
                    pub ready: PB5<Input<Floating>>, 
                    pub reset: PA0<Output<PushPull>>
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
   let clocks = rcc.clocks;  // not sure if this is right

   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let mut pin = gpiob.pb7.into_open_drain_output();
   pin.set_high().unwrap(); // Pull high to avoid confusing the sensor when initializing.

   let sda = gpiob.pb9.into_alternate_open_drain(); 
   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let i2c1 = dp.I2C1.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

   let sda = gpioa.pa8.into_alternate_open_drain(); 
   let scl = gpioc.pc4.into_alternate_open_drain();
   let i2c2 = dp.I2C2.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

   let mut led = gpioc.pc13.into_push_pull_output();
   led.off();

   let spi1 = dp.SPI1.spi(
       (gpioa.pa5.into_alternate(), // sck  
        gpioa.pa6.into_alternate(), // miso 
        gpioa.pa7.into_alternate(), // mosi 
       ),
       MODE, 8.MHz(), &mut rcc,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa1.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(), //ResetPin   
        };   

   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   //let (tx, rx) = Serial::new(dp.USART1,(tx, rx),...  would be nice
   let (tx, rx) = dp.USART1.usart(tx, rx, FullConfig::default().baudrate(115200.bps()),
         &mut rcc).unwrap().split();
   
   let timerx = Timer::new(dp.TIM3, &clocks);
   let delay = DelayFromCountDownTimer::new(timerx.start_count_down(100.millis()));

   (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}

