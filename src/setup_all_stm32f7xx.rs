
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, I2C1, I2C2, USART1, USART2, SPI1},
      spi::{Spi},
      pac::{I2C1, I2C2},
      i2c::I2c,
      i2c::I2c,   //this is a type
      serial::{Serial, Tx, Rx, Error},
      gpio::{Output, OpenDrain, PushPull},
      prelude::*,
};

use stm32f7xx_hal::{
      pac,
      gpio::{gpioa::PA8},
      serial::{Config, Oversampling, DataBits, Parity},
};

use embedded_hal::spi::{Mode, Phase, Polarity};

//   //////////////////////////////////////////////////////////////////////

pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = BlockingI2c<I2C1, PB8<Alternate<4u8, OpenDrain>>, PB9<Alternate<4u8, OpenDrain>>>;
pub type I2c1Type =  BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>;
pub type I2cType  = I2c1Type; 

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

pub type TxType = Tx<USART1>;
pub type RxType = Rx<USART1>;

pub type SpiType =  Spi<SPI1>;
pub struct SpiExt { pub cs:    Pin<'A', 1, Output>, 
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

//   //////////////////////////////////////////////////////////////////////


pub fn all_from_dp(dp: Peripherals) -> 
               (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, RxType, SpiType, SpiExt, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut dht   = gpioa .pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

   //let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);
    let i2c1 = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

    let scl = gpiob.pb10.into_alternate_open_drain(); 
    let sda = gpiob.pb11.into_alternate_open_drain(); 

    let i2c2 = BlockingI2c::i2c2(
        i2c2,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

   let mut led = setup_led(dp.GPIOC.split());
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

   let delay = dp.TIM2.delay_us(&clocks);

   let (tx, _rx) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9 
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        &clocks,
        Config {
            baud_rate: 115200.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
   )
   .split();

   (pin, i2c1, i2c2, led, tx, rx, spi1, spiext,  delay, clocks)
}

