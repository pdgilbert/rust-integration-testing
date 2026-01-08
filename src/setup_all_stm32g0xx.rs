pub use stm32g0xx_hal as hal;
pub use hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1,},
      rcc::{RccExt},
      spi::spi,
      //spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Input, Output, OpenDrain, PushPull, Floating, Analog, GpioExt},
      prelude::*,
      prelude,
      block,
};

pub use stm32g0xx_hal::{
    pac::{TIM2, TIM3, ADC},
    i2c::{Config, SDAPin, SCLPin},
    rcc::{Clocks},
    timer::delay::Delay as halDelay,
    spi::{Mode, Phase, Polarity},
    serial::{FullConfig},
    gpio::{gpioa::{PA1, PA4, PA8},
           gpiob::{PB4, PB5},
           gpioc::{PC13 as LEDPIN}},
    analog::adc::Adc,
};


/////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

pub type Delay1Type = halDelay<TIM2>;
pub type Delay2Type = halDelay<TIM3>;
pub type Delay = Delay1Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

pub type OpenDrainType = PA8<Output<OpenDrain>>;

//pub type I2c1Type = I2c<I2C1, SDAPin, SCLPin>;
//pub type I2c2Type = I2c<I2C2, SDAPin, SCLPin>;
//  requires [define_opaque(I2c1Type, I2c2Type)] attribute to defining function below.
pub type I2c1Type = I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>;
pub type I2c2Type = I2c<I2C2, impl SDAPin<I2C2>, impl SCLPin<I2C2>>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type Tx1Type = Tx<USART1, FullConfig>;
pub type Rx1Type = Rx<USART1, FullConfig>;
pub type Tx2Type = Tx<USART2, FullConfig>;
pub type Rx2Type = Rx<USART2, FullConfig>;

pub type TxType = Tx1Type;
pub type RxType = Rx1Type;

//pub type SpiType =  impl embedded_hal_bus::spi::DeviceError::Spi<SPI1>;
//pub type SpiType =  embedded_hal_bus::spi::DeviceError;
pub type SpiType =  Spi<SPI1>;


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

//pub struct SpiExt { pub cs:    Pin<'A', 1, Output>, 
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

pub type AdcSensor1Type = AdcSensor<PA1<Analog>, Adc>;


//   //////////////////////////////////////////////////////////////////////

#[define_opaque(I2c1Type, I2c2Type)]
pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.clocks; 
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc);

   let scl = gpiob.pb8.into_open_drain_output_in_state(PinState::High); 
   let sda = gpiob.pb9.into_open_drain_output_in_state(PinState::High); 
   let i2c1 = dp.I2C1.i2c(sda, scl, Config::with_timing(0x2020_151b), &mut rcc);

   let scl = gpiob.pb10.into_open_drain_output_in_state(PinState::High);
   let sda = gpiob.pb11.into_open_drain_output_in_state(PinState::High); 
   let i2c2 = dp.I2C2.i2c(sda, scl,  Config::with_timing(0x2020_151b), &mut rcc);

   let gpioc = dp.GPIOC.split(&mut rcc);
   //let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   let mut led = gpioc.pc13.into_push_pull_output();  //LEDPIN is pc13. This is awkward.
   led.off();

   //let spi1 = Spi::new(
   //    dp.SPI1,
   let spi1 = dp.SPI1.spi(
       (
           gpioa.pa5, //.into_alternate(), // sck  
           gpioa.pa6, //.into_alternate(), // miso 
           gpioa.pa7, //.into_alternate(), // mosi 
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

    let (tx1, rx1) = dp.USART1.usart((gpioa.pa9, gpioa.pa10),
                        FullConfig::default(), &mut rcc).unwrap().split();

    let (tx2, rx2) = dp.USART2.usart((gpioa.pa2, gpioa.pa3),
                        FullConfig::default(), &mut rcc).unwrap().split();



   let delay = dp.TIM2.delay(&mut rcc);

   

   let adc1: AdcSensor1Type = AdcSensor {
       ch:  gpioa.pa1.into_analog(),
       adc: dp.ADC.constrain(&mut rcc),
   }; 
   impl ReadAdc for AdcSensor1Type {
       fn read_mv(&mut self)    -> u32 { self.adc.read_voltage(&mut self.ch).unwrap() as u32}
   }

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

