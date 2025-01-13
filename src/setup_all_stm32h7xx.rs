pub use stm32h7xx_hal as hal;
pub use hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1, ADC1,},
      //timer::{Delay as halDelay},
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


pub use stm32h7xx_hal::{
      rcc::CoreClocks as Clocks,
      pac::{TIM2, TIM5},
      timer::Timer,
      //delay::Delay,
      delay::DelayFromCountDownTimer,
      spi::{Enabled}, // may need SpiExt from here, but name conflict
      adc,
      gpio::{Input,
             gpioa::{PA0, PA1, PA4, PA8},
             gpiob::{PB4, PB5, PB8, PB9},
             gpiof::{PF0, PF1},
             gpioc::PC13 as LEDPIN},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

#[cfg(feature = "stm32h7xx")]
pub type Delay1Type = DelayFromCountDownTimer<TIM2>;
//pub type Delay1Type = DelayFromCountDownTimer<CountDown<TIM2>>;
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;

#[cfg(feature = "stm32h7xx")]
pub type Delay2Type = DelayFromCountDownTimer<Timer<TIM5>>;
//pub type Delay2Type = DelayFromCountDownTimer<TIM5>;
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM5, 1000000_u32>;
//pub type Delay2Type = Delay;
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = DelayFromCountDownTimer<CountDown<TIM5>>;
//pub type Delay2Type = Delay<TIM5, 1000000_u32>;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM5, 1000000_u32>;

pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
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

pub type SpiType =  Spi<SPI1, Enabled>;

pub type Cs    = PA4<Output<PushPull>>; 
pub type Busy  = PB4<Input>;
pub type Ready = PB5<Input>;
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

pub type AdcSensor1Type = AdcSensor<PA0<Analog>, Adc<ADC1, adc::Enabled>>;

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

impl ReadAdc for AdcSensor1Type {
    fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
}

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
   let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
   let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
   let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   //let i2cx = ccdr.peripheral.I2C1;  //.I2C4;
   //let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
   let i2c1 =  dp.I2C1.i2c(
                    (gpiob.pb8.into_alternate().set_open_drain(), // scl  
                     gpiob.pb9.into_alternate().set_open_drain(), // sda
                    ), 400.kHz(), ccdr.peripheral.I2C1, &clocks);

   let i2c2 =  dp.I2C2.i2c(
                    (gpiof.pf1.into_alternate().set_open_drain(), // scl
                     gpiof.pf0.into_alternate().set_open_drain(), // sda
                    ), 400.kHz(), ccdr.peripheral.I2C2, &clocks);

   let mut led: LedType = gpioc.pc13.into_push_pull_output();
   led.off();

   let spi1 = dp.SPI1.spi(
       (gpioa.pa5.into_alternate(), // sck  
        gpioa.pa6.into_alternate(), // miso 
        gpioa.pa7.into_alternate(), // mosi 
       ),
       MODE, 8.MHz(), ccdr.peripheral.SPI1, &clocks,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa4.into_push_pull_output(), //CsPin       // UNTESTED    
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa1.into_push_pull_output(), //ResetPin   
        };   

   // CountDownTimer not supported by embedded-hal 1.0.0 ??
   let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
   let mut delay = DelayFromCountDownTimer::new(timer);

   let (tx1, rx1) = dp.USART1.serial(
            (
                gpioa.pa9.into_alternate(), //tx pa9
                gpioa.pa10.into_alternate(),
            ), //rx pa10
            115200.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split();

    let (tx2, rx2) = dp.USART2.serial(
            (
                gpioa.pa2.into_alternate(), //tx pa2
                gpioa.pa3.into_alternate(),
            ), //rx pa3
            9600.bps(),
            ccdr.peripheral.USART2,
            &clocks,
        )
        .unwrap()
        .split();


   
   let mut adcx = Adc::adc1(dp.ADC1, 4.MHz(), &mut delay, ccdr.peripheral.ADC12, &ccdr.clocks);
   adcx.set_resolution(adc::Resolution::SixteenBit);
   let adcx = adcx.enable();

   let adc1: AdcSensor1Type = AdcSensor {
       ch:  gpioa.pa0.into_analog(),
       adc: adcx,
   }; 

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

