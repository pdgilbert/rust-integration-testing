pub use nb::block;

pub use stm32f1xx_hal as hal;
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
      //block,
};

pub use stm32f1xx_hal::{
      pac::{TIM2, TIM3}, 
      rcc::Clocks,
      timer::{TimerExt, Delay as halDelay},
      i2c::{DutyCycle, Mode as i2cMode, BlockingI2c,},   // //Pins
      spi::{Mode, Phase, Polarity},
      //spi::{Spi1NoRemap, Pins as SpiPins, Error as SpiError},
      serial::{Config},
      gpio::{Alternate, Pin, Input, 
             gpioa::{PA0, PA1, PA8, PA11},
             gpiob::{PB0, PB1, PB6, PB7, PB8, PB9, PB10, PB11},
             gpioc::{PC13 as LEDPIN}},
};

//use embedded_hal::spi::{Mode, Phase, Polarity};
//note: `Mode` and `Mode` have similar names, but...
//use embedded_hal::i2c::ErrorType;


//   //////////////////////////////////////////////////////////////////////
pub use embedded_hal::delay::DelayNs;

//pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay;
//pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay;

pub type Delay1Type = halDelay<TIM2, 1000000_u32>;
pub type Delay2Type = halDelay<TIM3, 1000000_u32>;

pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub type OpenDrainType = PA8<Output<OpenDrain>>;

//pub type I2c1Type = I2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>;
//pub type I2c1Type = I2c<I2C1, impl Pins<I2C1> >;
//pub type I2c2Type = I2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>;
//pub type I2c2Type = I2c<I2C2, impl Pins<I2C2> >;
pub type I2c1Type = BlockingI2c<I2C1>;
pub type I2c2Type = BlockingI2c<I2C2>;
//pub type I2c1Type = I2c<I2C1, Error = ErrorType>;
//pub type I2c2Type = I2c<I2C2, Error = ErrorType>;
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

//pub type SpiType =  Spi<SPI1, Spi1NoRemap, impl SpiPins<Spi1NoRemap>>;
pub type SpiType =  Spi<SPI1, u8>;
//pub struct SpiExt { pub cs:    Pin<'A', 11, Output>, //UNTESTED
//                    pub busy:  Pin<'B', 8>, 
//                    pub ready: Pin<'B', 9>, 
//                    pub reset: Pin<'A', 0, Output>
//pub struct SpiExt { pub cs:    Pin<'A', 11, Output>, //UNTESTED
//                    pub busy:  Pin<'B', 0>, 
//                    pub ready: Pin<'B', 1>, 
//                    pub reset: Pin<'A', 0, Output>
//}
// these should just be in SpiExt, but radio Sx127x still wants them separately
pub type Cs    = PA11<Output<PushPull>>;
pub type Busy  = PB0<Input<>>;
pub type Ready = PB1<Input<>>;
pub type Reset = PA0<Output<PushPull>>;

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

pub type AdcSensor1Type = AdcSensor<PA1<Analog>, Adc<ADC1>>;

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

impl ReadAdc for AdcSensor1Type {
       fn read_mv(&mut self)    -> u32 { 
          self.adc.read(&mut self.ch).unwrap() 
       } 
   }


//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    //let mut afio = dp.AFIO.constrain();
    let clocks = rcc
        .cfgr
        //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
        //.sysclk(64.mhz()) // system clock 8 MHz default, max 72MHz
        //.pclk1(32.mhz())  // system clock 8 MHz default, max 36MHz ?
        .freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    //hprintln!("hclk {:?}",   clocks.hclk()).unwrap();
    //hprintln!("sysclk {:?}", clocks.sysclk()).unwrap();
    //hprintln!("pclk1 {:?}",  clocks.pclk1()).unwrap();
    //hprintln!("pclk2 {:?}",  clocks.pclk2()).unwrap();
    //hprintln!("pclk1_tim {:?}", clocks.pclk1_tim()).unwrap();
    //hprintln!("pclk2_tim {:?}", clocks.pclk2_tim()).unwrap();
    //hprintln!("adcclk {:?}",    clocks.adcclk()).unwrap();
    ////hprintln!("usbclk_valid {:?}", clocks.usbclk_valid()).unwrap(); not fo all MCUs

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

    //afio  needed for alt remap of (PB8, PB9) but not (PB6,PB7) because they are not alt

    //let i2c1 = BlockingI2c::<I2C1>::new(
    //    dp.I2C1,
    //    (
    //     gpiob.pb6, 
    //     gpiob.pb7
    //    ),
    //    i2cMode::Fast {
    //        frequency: 100_000_u32.Hz(),
    //        duty_cycle: DutyCycle::Ratio2to1,
    //    },
    //    &clocks,  //1000, 10, 1000, 1000,
    //);

    let mut afio = dp.AFIO.constrain();

    // still only on branch = "rmp-new"
    let i2c1 = BlockingI2c::<I2C1>::new(
                  dp.I2C1
                  .remap(&mut afio.mapr),  // add this for PB8, PB9
                  (
                   gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh),  // scl 
                   gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh)   // sda
                  ),
                  i2cMode::Fast {frequency: 400.kHz(), duty_cycle: DutyCycle::Ratio16to9,},
                  &clocks, 1000, 10, 1000, 1000,);
    // or
    //let i2c1 = dp
    //    .I2C1
    //    .remap(&mut afio.mapr) // add this for PB8, PB9
    //    .blocking_i2c(
    //      (
    //       gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh), 
    //       gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh)
    //      ),
    //      Mode::Fast {frequency: 400.kHz(), duty_cycle: DutyCycle::Ratio16to9, },
    //      &clocks, 1000, 10, 1000, 1000,
    //    );
 

    let i2c2 = BlockingI2c::<I2C2>::new(
                 dp.I2C2,
                 (
                  gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl 
                  gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda
                 ),
                 i2cMode::Fast {frequency: 400_000_u32.Hz(), duty_cycle: DutyCycle::Ratio2to1,},
                 &clocks, 1000, 10, 1000, 1000,);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.off();

   let spi1 =  dp.SPI1
        //.remap(&mut afio.mapr) // if you want to use PB3, PB4, PB5
        .spi((
           Some(gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl)), //   sck 
           Some(gpioa.pa6.into_floating_input(&mut gpioa.crl)),      //   miso
           Some(gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl)), //   mosi
        ),
       //&mut afio.mapr,
       MODE,
       8.MHz(), 
       &clocks,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa11.into_push_pull_output(&mut gpioa.crh), //CsPin         //pa11 UNTESTED
        busy:  gpiob.pb0.into_floating_input(&mut gpiob.crl),   //BusyPin  DI00 
        ready: gpiob.pb1.into_floating_input(&mut gpiob.crl),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(&mut gpioa.crl), //ResetPin   
        };   

    //let delay = DelayType{};
    let delay = dp.TIM3.delay::<1000000_u32>(&clocks);

    let (tx1, rx1) = dp.USART1
                       .serial(
                           (gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
                            gpioa.pa10,
                           ),
                           Config::default().baudrate(115200.bps()), 
                           &clocks,
                        ).split();
    
    let (tx2, rx2) = Serial::new( 
        dp.USART2,
        (
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl), 
            gpioa.pa3,  // probably need alt
        ), 
        Config::default().baudrate(9_600.bps()),
        &clocks,
    )
    .split();
    

   let adc1: AdcSensor1Type = AdcSensor {
        ch:  gpioa.pa1.into_analog(&mut gpioa.crl), //channel
        adc: Adc::adc1(dp.ADC1, &clocks),
   }; 

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

