
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      pac::{I2C1, I2C2},
      i2c::I2c,
      serial::{Serial, Tx, Error},
      gpio::{gpioa::PA8, Output, OpenDrain},
      prelude::*,
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
//pub type I2c1Type = I2c<I2C1, PB8<Alternate<AF1>>, PB7<Alternate<AF1>>>;
//pub type I2c1Type = I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
pub type TxType = Tx<USART1>;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
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

   let delay = DelayType{};
   //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &rcc);

   let (tx, rx) = cortex_m::interrupt::free(move |cs| {
       (
           gpioa.pa9.into_alternate_af1(cs),  //tx pa9
           gpioa.pa10.into_alternate_af1(cs), //rx pa10
       )
   });

   let (tx, _rx) = Serial::usart1(dp.USART1, (tx, rx), 9600.bps(), &mut rcc).split();

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

