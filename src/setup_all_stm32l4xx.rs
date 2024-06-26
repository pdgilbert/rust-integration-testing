pub use stm32l4xx_hal as hal;
pub use hal::{
      pac::CorePeripherals,   //hopefully temperary, used in some examples
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1, ADC1,},
      timer::{Delay as halDelay},
      rcc::{RccExt},
      spi::{Spi},
      pac::{I2C1, I2C2},
      i2c::I2c,
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull, Analog, GpioExt},
      adc::Adc,
      prelude::*,
      prelude,
      block,
};

pub use stm32l4xx_hal::{
    serial::{Config as serialConfig, },
    gpio::{gpioa::PA8, PA11},
    gpio::{gpioc::{PC13 as LEDPIN}},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub use embedded_hal::delay::DelayNs;

pub use crate::alt_delay::{AltDelay as Delay1Type};
//pub type Delay1Type = Delay; //<TIM2, 1000000_u32>;
pub use crate::alt_delay::{AltDelay as Delay2Type};
//pub type Delay2Type = Delay; //<TIM3, 1000000_u32>;
pub type Delay = Delay2Type;

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, (PB8<Alternate<OpenDrain, 4u8>>, PB9<Alternate<OpenDrain, 4u8>>)>;
pub type I2c1Type =  I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>;
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

pub type AdcSensor1Type = AdcSensor<PA1<Analog>, Adc<ADC1>>;


//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type, Rx1Type, Tx2Type, Rx2Type, 
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

   let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
   let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);


   let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
    let mut scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let mut sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c1 = I2c::i2c1(i2c1, (scl, sda), Config::new(400_u32.kHz(), clocks), apb1r1 );

    let mut scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let mut sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c2 = I2c::i2c2(i2c2, (scl, sda), Config::new(400_u32.kHz(), clocks),  apb1r1 );

   let mut led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
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

   let (tx1, rx1) = Serial::usart1(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        Config::default().baudrate(115200.bps()),
        clocks,
        &mut rcc.apb2,
   )
   .split();

   let (tx2, rx2) = Serial::usart2(
        dp.USART2,
        (
            gpioa
                .pa2
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2  for GPS
            gpioa
                .pa3
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3  for GPS
        ),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

   

   let adc1: AdcSensor1Type = AdcSensor {
       ch:  gpioa.pa1.into_analog(),
       adc: dp.ADC1.claim(ClockSource::SystemClock, &rcc, &mut delay, true),
   }; 
   impl ReadAdc for AdcSensor1Type {
       fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() as u32 }
   }

   (pin, i2c1, i2c2, led, tx1, rx1,  tx2, rx2, spi1, spiext,  delay, clocks, adc1)
}

