use stm32f4xx_hal as hal;
pub use hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1, ADC1,},
      timer::{Delay as halDelay},
      rcc::{RccExt},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull, Analog, GpioExt, Input},
      adc::Adc,
      prelude::*,
      prelude,
      block,
};

// above are commom to all hals. Below are possibly different.

pub use stm32f4xx_hal::{
    pac::{TIM2, TIM5},
    rcc::{Clocks},
    timer::{TimerExt},
    serial::{config::Config},
    gpio::{Pin}, 
    gpio::{gpioa::{PA0, PA1, PA8, PA4, PA5, PA6, PA7}},
    gpio::{gpiob::{PB4, PB5}},
    gpio::{gpioc::{PC13 as LEDPIN}},
    adc::{config::{AdcConfig, SampleTime}},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

pub type Delay1Type = halDelay<TIM2, 1000000_u32>;
pub type Delay2Type = halDelay<TIM5, 1000000_u32>;
pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

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
pub type Tx2Type = Tx<USART2>;
pub type Rx2Type = Rx<USART2>;

pub type TxType = Tx1Type;
pub type RxType = Rx1Type;

pub type SpiType =  Spi<SPI1>;
//pub struct SpiExt { pub cs:    Pin<'A', 4, Output>, 
//                    pub busy:  Pin<'B', 4>, 
//                    pub ready: Pin<'B', 5>, 
//                    pub reset: Pin<'A', 1, Output>
//}
// these should just be in SpiExt, but radio Sx127x still wants them separately
pub type Cs    = PA4<Output<PushPull>>;
pub type Busy  = PB4<Input<>>;
pub type Ready = PB5<Input<>>;
pub type Reset = PA1<Output<PushPull>>;

pub struct SpiExt { pub cs:    Cs, 
                    pub busy:  Busy, 
                    pub ready: Ready, 
                    pub reset: Reset
}


// this really should be set in example code
pub const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub struct AdcSensor<U, A> { ch: U, adc: A }

pub type AdcSensor1Type = AdcSensor<PA0<Analog>, Adc<ADC1>>; 

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

impl ReadAdc for AdcSensor1Type {
    fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() as u32 }
}


//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) -> 
          (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
   let mut rcc = dp.RCC.constrain();

   // according to  https://github.com/rtic-rs/rtic/blob/master/examples/stm32f411_rtc_interrupt/src/main.rs
   // 25 MHz must be used for HSE on the Blackpill-STM32F411CE board according to manual
   // let clocks = rcc.cfgr.use_hse(25.MHz()).freeze();

   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.


   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let sda = gpiob.pb9.into_alternate_open_drain(); 
   let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &mut rcc);

   let scl = gpiob.pb10.into_alternate_open_drain();
   let sda = gpiob.pb3.into_alternate_open_drain();
   let i2c2 = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &mut rcc);

   let mut led = gpioc.pc13.into_push_pull_output();
   led.off();

   let spi1 = Spi::new(
       dp.SPI1,
       (
           Some(gpioa.pa5.into_alternate()), // sck  
           Some(gpioa.pa6.into_alternate()), // miso 
           Some(gpioa.pa7.into_alternate()), // mosi 
       ),
       MODE, 8.MHz(), &mut rcc,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa4.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa1.into_push_pull_output(), //ResetPin   
        };   

   let (tx1, rx1) = Serial::new(dp.USART1, 
                      (gpioa.pa9.into_alternate(),
                       gpioa.pa10.into_alternate()
                      ),
                      Config::default().baudrate(115200.bps()), &mut rcc).unwrap().split();

    let (tx2, rx2) = Serial::new( dp.USART2,
                       (gpioa.pa2.into_alternate(), 
                        gpioa.pa3.into_alternate(),
                       ),
                       Config::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

   let delay = dp.TIM5.delay(&mut rcc);

   let adc1: AdcSensor1Type = AdcSensor {
        ch:  gpioa.pa0.into_analog(), //channel
        adc: Adc::new(dp.ADC1, true, AdcConfig::default(), &mut rcc),
   }; 

   //let clocks = rcc.cfgr().freeze();

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay,  rcc.clocks, adc1)
}


