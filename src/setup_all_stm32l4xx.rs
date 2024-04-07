
pub use stm32l4xx_hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      pac::{I2C1, I2C2},
      i2c::I2c,
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32l4xx_hal::{
      serial::{Config as serialConfig, },
      gpio::{gpioa::PA8},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::led::{setup_led, LED, LedType};

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;
//pub type I2c1Type = I2c<I2C1, (PB8<Alternate<OpenDrain, 4u8>>, PB9<Alternate<OpenDrain, 4u8>>)>;
//pub type I2c1Type =  I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>;
pub type TxType = Tx<USART1>;

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
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

   let delay = DelayType{};

   let (tx, _rx) = Serial::usart1(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9  for console
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10 for console
        ),
        Config::default().baudrate(115200.bps()),
        clocks,
        &mut rcc.apb2,
   )
   .split();

   (pin, i2c1, i2c2, led, tx, delay, clocks)
}

