use stm32g4xx_hal as hal;
pub use hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1, ADC1,},
      rcc::{RccExt},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull, Analog, GpioExt},
      adc::Adc,
      prelude::*,
      prelude,
      block,
};

// above are commom to all hals. Below are different.

pub use stm32g4xx_hal::{
    pac::{TIM2, TIM3}, 
    i2c::{Config, SDAPin, SCLPin},
    rcc::{Clocks},
    time::{ExtU32, RateExtU32},
    timer::Timer,
    timer::CountDownTimer,
    delay::DelayFromCountDownTimer,
    spi::{Mode, Phase, Polarity},
    serial::{FullConfig, NoDMA},
    gpio::{Alternate, Input, 
           gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10},
           gpiob::{PB4, PB5, PB7, PB8, PB9},
           gpioc::{PC4, PC6 as LEDPIN}},  // weact-stm32g474CEU6 has onboard led on PC6
    adc::{config::{SampleTime, AdcConfig}, Disabled, Configured, AdcClaim, AdcCommonExt},
};


//   //////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

pub type Delay1Type = DelayFromCountDownTimer<CountDownTimer<TIM2>>;
pub type Delay2Type = DelayFromCountDownTimer<CountDownTimer<TIM3>>;
pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

pub type OpenDrainType = PB7<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>;
pub type I2c2Type = I2c<I2C2, impl SDAPin<I2C2>, impl SCLPin<I2C2>>;
// These need nightly and #![feature(type_alias_impl_trait)]
// As of 1.89.0-nightly (circa May-June 2025) the change to  unstable TAITs (type alias impl trait) 
//  requires [define_opaque(I2c1Type, I2c2Type)] attribute to defining function below.
//   See link mentioned in https://github.com/stm32-rs/stm32g4xx-hal/issues/211
//pub type I2c1Type = I2c<I2C1, PB9<I2C1>, PB8<I2C1>>;
//pub type I2c2Type = I2c<I2C2, PA8<I2C2>, PC4<I2C2>>;
//pub type I2c1Type = I2c<I2C1>;
//pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}

pub type Tx1Type = Tx<USART1, PA9<Alternate<7_u8>>, NoDMA>;
pub type Rx1Type = Rx<USART1, PA10<Alternate<7_u8>>, NoDMA>;
pub type Tx2Type = Tx<USART2, PA2<Alternate<7_u8>>, NoDMA>; 
pub type Rx2Type = Rx<USART2, PA3<Alternate<7_u8>>, NoDMA>;

pub type TxType = Tx1Type;
pub type RxType = Rx1Type;

pub type SpiType =  Spi<SPI1,(PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>;
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

pub type AdcSensor1Type = AdcSensor<PA0<Analog>, Adc<ADC1, Configured>>; 

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

impl ReadAdc for AdcSensor1Type {
    fn read_mv(&mut self)    -> u32 { 
       let sample = self.adc.convert(&self.ch, SampleTime::Cycles_640_5);
       self.adc.sample_to_millivolts(sample) as u32
    }
}

//   //////////////////////////////////////////////////////////////////////

#[define_opaque(I2c1Type, I2c2Type)]
pub fn all_from_dp(dp: Peripherals) -> 
          (OpenDrainType, 
           I2c1Type,  //I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>, 
           I2c2Type,  //I2c<I2C2, impl SDAPin<I2C2>, impl SCLPin<I2C2>>, 
           LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.clocks; 

   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);
   let gpioc = dp.GPIOC.split(&mut rcc);

   let mut pin = gpiob.pb7.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let sda = gpiob.pb9.into_alternate_open_drain(); 
   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let i2c1 = dp.I2C1.i2c(sda, scl, Config::new(400.kHz()), &mut rcc); // NOTE ORDER OF SDA,SCL REVERSED FROM stm32f4xx

   let sda = gpioa.pa8.into_alternate_open_drain(); 
   let scl = gpioc.pc4.into_alternate_open_drain();
   let i2c2 = dp.I2C2.i2c(sda, scl, Config::new(400.kHz()), &mut rcc); // NOTE ORDER OF SDA,SCL REVERSED FROM stm32f4xx

   let mut led = gpioc.pc6.into_push_pull_output();
   led.off();

   let spi1 = dp.SPI1.spi(
       (gpioa.pa5.into_alternate(), // sck  
        gpioa.pa6.into_alternate(), // miso 
        gpioa.pa7.into_alternate(), // mosi 
       ),
       MODE, 8.MHz(), &mut rcc,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa4.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa1.into_push_pull_output(), //ResetPin   
        };   

   let (tx1, rx1) = dp.USART1.usart(
                        gpioa.pa9.into_alternate(),  //tx, 
                        gpioa.pa10.into_alternate(), //rx, 
                        FullConfig::default().baudrate(115200.bps()),
                        &mut rcc).unwrap().split();
   
   let (tx2, rx2) = dp.USART2.usart(   
                        gpioa.pa2.into_alternate(),  //tx
                        gpioa.pa3.into_alternate(),  //rx 
                        FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

   let timerx = Timer::new(dp.TIM3, &clocks);
   let mut delay = DelayFromCountDownTimer::new(timerx.start_count_down(100.millis()));

   let adc12_common = dp.ADC12_COMMON.claim(Default::default(), &mut rcc);
   let adc1: AdcSensor1Type = AdcSensor {
       ch:  gpioa.pa0.into_analog(),
       adc: adc12_common.claim_and_configure(dp.ADC1, AdcConfig::default(), &mut delay),
   }; 

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

