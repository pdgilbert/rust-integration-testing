
pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      pac::{I2C1, I2C2},
      i2c::I2c,
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32f3xx_hal::{
    gpio::{gpioa::{PA8, PA9}, PushPull, AF7 },
};

//   //////////////////////////////////////////////////////////////////////


pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
//pub type I2c1Type = I2c<I2C1, (PB6<AF4<OpenDrain>>, PB7<AF4<OpenDrain>>)>;
//pub type I2c1Type = I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)> ;

pub type TxType = Tx<USART1, PA9<AF7<PushPull>>>;
//pub type TxType = Tx<USART1, impl TxPin<USART1>>;  // impl is unstable in type alias
// See  https://github.com/stm32-rs/stm32f3xx-hal/issues/288
//   regarding why it is necessary to specify the concrete pin here.

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;

impl LED for LedType {  // none default
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }


//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
   let mut flash = dp.FLASH.constrain();
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze(&mut flash.acr);

   let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
   let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc.ahb);
   //let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);
    let scl = gpiob.pb6.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    //    // //NOT sure if pull up is needed
    //    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c1 = I2c::new(i2c1, (scl, sda), 100_000.Hz(), clocks, &mut apb1);

    let scl =  gpioa.pa9.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let sda = gpioa.pa10.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    //    // //NOT sure if pull up is needed
    //    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c2 = I2c::new(i2c2, (scl, sda), 100_000.Hz(), clocks, &mut apb1);

   let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
   led.off();

   let delay = DelayType{};

   let (tx, _rx) = Serial::new(
       dp.USART1,
       (
           gpioa
               .pa9
               .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
           gpioa
               .pa10
               .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
       ),
       115200.Bd(),
       clocks,
       &mut rcc.apb2,
   )
   .split();

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

