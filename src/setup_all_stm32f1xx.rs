
pub use crate::stm32f1xx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      pac::{I2C1, I2C2},
      i2c::I2c,
      serial::{Serial, Tx, Error},
      gpio::{Output, OpenDrain},
      prelude::*,
};

use stm32f1xx_hal::{
      rcc::Clocks,
      gpio::{gpioa::PA8},
      serial::{Config},
};


//   //////////////////////////////////////////////////////////////////////

pub use crate::delay::{Delay2Type as Delay};

pub type OpenDrainType = PA8<Output<OpenDrain>>;

pub type I2c1Type = I2cType<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;
//pub type I2c1Type = I2cType<I2C1, impl Pins<I2C1> >;
pub type I2c2Type = I2cType<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>;
//pub type I2c2Type = I2cType<I2C2, impl Pins<I2C2> >;

pub type TxType = Tx<USART1>;

pub use crate::led::LED;  // defines trait and default methods
pub type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

//   //////////////////////////////////////////////////////////////////////

pub fn all_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
    let clocks = rcc
        .cfgr
        //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
        //.sysclk(72.mhz()) // system clock 8 MHz default, max 72MHz
        //.pclk1(36.mhz())  // system clock 8 MHz default, max 36MHz ?
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
    let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

    let gpiob = dp.GPIOB.split();

    //afio  needed for i2c1 (PB8, PB9) but not i2c2
    //let i2c = setup_i2c1(dp.I2C1, gpiob, &mut afio, &clocks);
    let i2c1 = I2c::i2c1(
        i2c1,
        (gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh), 
         gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh)
        ),
        &mut afio.mapr,
        Mode::Fast {
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
        i2c2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 (PB8, PB9) but //NOT i2c2
        Mode::Fast {
            frequency: 400_000_u32.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        //1000,
        //10,
        //1000,
        //1000,
    );

    let mut led = setup_led(dp.GPIOC.split()); 
    led.off();

    //let delay = DelayType{};
    let delay = dp.TIM5.delay(&clocks);

    // NOTE, try to figure out the proper way to deal with this:
    // Using gpiob (PB6-7) for serial causes a move problem because gpiob is also used for i2c.
    // (There can also a move problem with afio if setup_i2c1() takes afio rather than &mut afio,
    // but that can be resolved by just doing serial() before setup_i2c1().)

    let (tx, _rx) = Serial::new(
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

    (pin, i2c1, i2c2, led, tx, delay, clocks)
}

