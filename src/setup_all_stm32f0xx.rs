pub use stm32f0xx_hal as hal;
pub use hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1, ADC1,},
      rcc::{RccExt},
      spi::{Spi},
      pac::{I2C1, I2C2},
      i2c::I2c,
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{gpioa::{PA8, PA11}, Output, OpenDrain},
      adc::Adc,
      prelude::*,
      prelude,
      block,
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay;

pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay;

pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, PB8<Alternate<AF1>>, PB7<Alternate<AF1>>>;
pub type I2c1Type = I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>;
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

pub type SpiType =  Spi<SPI1>;
pub struct SpiExt { pub cs:    Pin<'A', 11, Output>,   //pa11 UNTESTED
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

pub struct AdcSensor<U, A> { ch: U, adc: A }

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

pub type AdcSensor1Type = AdcSensor<PA1<Analog>, Adc>;


//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
   let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
   let gpioa = dp.GPIOA.split(&mut rcc);

   let mut pin = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));
   pin.set_high().ok();

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc),  &mut rcc);
    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (gpiob.pb8.into_alternate_af1(cs), // scl on PB8
         gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });
    let i2c1 = I2c::i2c1(i2c1, (scl, sda), 400.khz(), rcc);

    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (gpiob.pb10.into_alternate_af1(cs),
         gpiob.pb11.into_alternate_af1(cs),
        )
    });
    let i2c2 = I2c::i2c2(i2c2, (scl, sda), 400.khz(), rcc);


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
        cs:    gpioa.pa11.into_push_pull_output(), //CsPin    //pa11 UNTESTED       
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(), //ResetPin   
        };   

   let delay = DelayType{};
   //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &rcc);

   let (tx, rx) = cortex_m::interrupt::free(move |cs| {
       (
           gpioa.pa9.into_alternate_af1(cs),  //tx pa9
           gpioa.pa10.into_alternate_af1(cs), //rx pa10
       )
   });
   let (tx1, rx1, tx2, rx2) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa9.into_alternate_af1(cs),  //tx pa9
            gpioa.pa10.into_alternate_af1(cs), //rx pa10
            gpioa.pa2.into_alternate_af1(cs),  //tx pa2
            gpioa.pa3.into_alternate_af1(cs),  //rx pa3
        )
    });

   let (tx1, rx1) = Serial::usart1(p.USART1, (tx1, rx1), 9600.bps(), &mut rcc).split();
   
   let (tx2, rx2) = Serial::usart2(p.USART2, (tx2, rx2), 9600.bps(), &mut rcc).split();

   let adc1: AdcSensor1Type = AdcSensor {
       ch:  gpioa.pa1.into_analog(),
       adc: dp.ADC1.claim(ClockSource::SystemClock, &rcc, &mut delay, true),
   }; 
   impl ReadAdc for AdcSensor1Type {
       fn read_mv(&mut self)    -> u32 {  self.adc.read(&mut self.ch).unwrap() }
   }

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

