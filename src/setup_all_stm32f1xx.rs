pub use stm32f1xx_hal as hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

pub use stm32f1xx_hal::{
      rcc::Clocks,
      timer::{TimerExt},  // Delay,
      i2c::{DutyCycle, Mode as i2cMode},   //BlockingI2c, //Pins
      spi::{Spi1NoRemap, Pins as SpiPins, Error as SpiError},  //Mode
      serial::{Config},
      gpio::{Alternate, Pin,
             gpioa::{PA8},
             gpiob::{PB6, PB7, PB8, PB9, PB10, PB11},
             gpioc::{PC13 as LEDPIN}},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>;
//pub type I2c1Type = I2c<I2C1, impl Pins<I2C1> >;
pub type I2c2Type = I2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>;
//pub type I2c2Type = I2c<I2C2, impl Pins<I2C2> >;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type TxType = Tx<USART1>;
pub type RxType = Rx<USART1>;

pub type SpiType =  Spi<SPI1, Spi1NoRemap, impl SpiPins<Spi1NoRemap>, u8>;
pub struct SpiExt { pub cs:    Pin<'A', 1, Output>, 
                    pub busy:  Pin<'B', 8>, 
                    pub ready: Pin<'B', 9>, 
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
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
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

    //afio  needed for i2c1 (PB8, PB9) but not i2c2
    //let i2c1 = BlockingI2c::i2c1(
    let i2c1 = I2c::i2c1(
        dp.I2C1,
        (gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl), 
         gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl)
        ),
        &mut afio.mapr,
        i2cMode::Fast {
            frequency: 100_000_u32.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        //1000,
        //10,
        //1000,
        //1000,
    );

    let i2c2 = I2c::i2c2(
        dp.I2C2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 (PB8, PB9) but //NOT i2c2
        i2cMode::Fast {
            frequency: 400_000_u32.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        //1000,
        //10,
        //1000,
        //1000,
    );

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.off();

   let spi1 =  Spi::spi1(
       dp.SPI1,
        (gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl), //   sck 
         gpioa.pa6.into_floating_input(&mut gpioa.crl),      //   miso
         gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl), //   mosi
        ),
       &mut afio.mapr,
       MODE, 8.MHz(), clocks,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa1.into_push_pull_output(&mut gpioa.crl), //CsPin         
        busy:  gpiob.pb8.into_floating_input(&mut gpiob.crh),   //BusyPin  DI00 
        ready: gpiob.pb9.into_floating_input(&mut gpiob.crh),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(&mut gpioa.crl), //ResetPin   
        };   

    //let delay = DelayType{};
    //let delay = dp.TIM5.delay(&clocks);
    let delay = dp.TIM3.delay::<1000000_u32>(&clocks);

    // NOTE, try to figure out the proper way to deal with this:
    // Using gpiob (PB6-7) for serial causes a move problem because gpiob is also used for i2c.
    // (There can also a move problem with afio if setup_i2c1() takes afio rather than &mut afio,
    // but that can be resolved by just doing serial() before setup_i2c1().)

    let (tx, rx) = Serial::new(
       dp.USART1,
       // (gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl), 
       //  gpiob.pb7,
       (gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
        gpioa.pa10,
       ),
       &mut afio.mapr,
       Config::default().baudrate(115200.bps()), 
       &clocks,
    ).split();

    (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}

