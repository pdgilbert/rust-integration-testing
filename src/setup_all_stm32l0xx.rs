pub use stm32l0xx_hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1, TIM2, TIM3, ADC as ADC1,},
      delay::{Delay as halDelay},
      rcc::{RccExt},
      rcc, // for ::Config but note name conflict with serial
      rcc::Clocks,
      spi::{Spi, SpiExt as halSpiExt},
      spi,
      spi::{Mode, Phase, Polarity},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      serial::{Config, Serial1Ext, },
      gpio::{Output, OpenDrain, PushPull, Input, Floating, Analog, GpioExt},
      gpio::{gpioa::{PA1, PA4, PA5, PA6, PA7, PA8},
             gpiob::{PB4, PB5, PB8, PB9, PB10, PB11},
             gpioc::{PC13 as LEDPIN}},
      adc::Adc,
      rtc::ClockSource,
      prelude::*,
      prelude,
 //     block,
};

//pub use embedded_time::rate::Hertz;
//pub use embedded_io::Hertz;


//use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;
pub use embedded_hal::digital::OutputPin;

//pub type Delay1Type = halDelay<TIM2>;
//pub type Delay2Type = halDelay<TIM3>;
pub type Delay1Type = halDelay<>;
pub type Delay2Type = halDelay<>;
pub type Delay = Delay1Type;

//pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay<>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay<>;
//pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, PB9<Output<OpenDrain>>,  PB8<Output<OpenDrain>>>;
pub type I2c2Type = I2c<I2C2, PB11<Output<OpenDrain>>, PB10<Output<OpenDrain>>>;
//pub type I2c2Type = I2c<I2C2, SDA, SCL>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type Tx1Type = Tx<USART1>;
pub type Rx1Type = Rx<USART1>;
pub type Tx2Type = Tx<USART2>;
pub type Rx2Type = Rx<USART2>;

pub type TxType = Tx1Type;
pub type RxType = Rx1Type;

pub type SpiType =  Spi<SPI1, (PA5<Analog>, PA6<Analog>, PA7<Analog>)>;
//pub type SpiType =  Spi<SPI1,(PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;

// these should just be in SpiExt, but radio Sx127x still wants them separately
pub type Cs    = PA4<Output<PushPull>>;
pub type Busy  = PB4<Input<Floating>>;
pub type Ready = PB5<Input<Floating>>;
pub type Reset = PA1<Output<PushPull>>;

pub struct SpiExt { pub cs:    Cs, 
                    pub busy:  Busy, 
                    pub ready: Ready, 
                    pub reset: Reset
}

//pub struct SpiExt { pub cs:    Pin<'A', 11, Output>,   //pa11 UNTESTED
//                    pub busy:  Pin<'B', 4>, 
//                    pub ready: Pin<'B', 5>, 
//                    pub reset: Pin<'A', 0, Output>
//}


// this really should be set in example code
pub const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub struct AdcSensor<U, A> { ch: U, adc: A }

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

pub type AdcSensor1Type = AdcSensor<PA1<Analog>, Adc<Ready>>;


//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
   let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
   let clocks = rcc.clocks;

   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8 type `PB8<Output<OpenDrain>>`
   let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
   let i2c1 = dp.I2C1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

   let scl = gpiob.pb10.into_open_drain_output(); 
   let sda = gpiob.pb11.into_open_drain_output();
   let i2c2 = dp.I2C2.i2c(sda, scl, 400_000.Hz(), &mut rcc);

   let (tx1, rx1) = dp.USART1.usart(
       gpioa.pa9,
       gpioa.pa10,
       Config::default().baudrate(115200.Bd()),
       &mut rcc,
   )
   .unwrap()
   .split();

   let (tx2, rx2) = dp.USART2.usart(
           gpioa.pa2, 
           gpioa.pa3, 
           Config::default().baudrate(9600.Bd()),
           &mut rcc,
       )
       .unwrap()
       .split();

   let led = gpioc.pc13.into_push_pull_output(); 
   led.off();

   let spi1 = dp.SPI1.spi(
      (
          gpioa.pa5, // sck  
          gpioa.pa6, // miso 
          gpioa.pa7, // mosi 
      ),
      MODE,
      8.MHz(),
      &mut rcc,
   );

   
   let spiext = SpiExt {
       cs:    gpioa.pa4.into_push_pull_output(), //CsPin       
       busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
       ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
       reset: gpioa.pa1.into_push_pull_output(), //ResetPin   
       };   

   //let delay = Delay1Type{};
   //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);
   let delay = dp.TIM2.delay(&mut rcc);

   

   let adc1: AdcSensor1Type = AdcSensor {
       ch:  gpioa.pa1.into_analog(),
       adc: dp.ADC.constrain(ClockSource::SystemClock),
       //adc: dp.ADC.constrain(ClockSource::SystemClock, &rcc, &mut delay, true),
   }; 
   impl ReadAdc for AdcSensor1Type {
       fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
   }

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

