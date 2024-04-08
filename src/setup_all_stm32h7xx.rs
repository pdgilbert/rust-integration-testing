pub use stm32h7xx_hal as hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      spi::{Spi},
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};


pub use stm32h7xx_hal::{
      rcc::CoreClocks as Clocks,
      spi::{Enabled}, // may need SpiExt from here, but name conflict
      delay::DelayFromCountDownTimer,
      gpio::{Input, GpioExt,
             gpioa::{PA0, PA1, PA8},
             gpiob::{PB4, PB5, PB8, PB9},
             gpiof::{PF0, PF1},
             gpioc::PC13 as LEDPIN},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2c<I2C1>;
pub type I2c2Type = I2c<I2C2>;
pub type I2cType  = I2c1Type;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type TxType  = Tx<USART1>;
pub type RxType = Rx<USART1>;

pub type SpiType =  Spi<SPI1, Enabled>;
pub struct SpiExt { pub cs:    PA1<Output<PushPull>>, 
                    pub busy:  PB4<Input>, 
                    pub ready: PB5<Input>, 
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
        cs:    gpioa.pa1.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(), //ResetPin   
        };   

   // CountDownTimer not supported by embedded-hal 1.0.0 ??
   let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   let (tx, rx) = dp.USART1.serial(
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


   (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}

