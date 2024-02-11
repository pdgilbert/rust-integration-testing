
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::delay::DelayNs;

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

pub use crate::delay::{Delay2Type as Delay};

pub use crate::monoclock::{MONOCLOCK};


// "stm32xxxx_hal" is used for items that are different in some crates
// "hal" is used for items that are the same in all hal  crates

pub use crate::stm32xxx_as_hal::hal;

pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{gpioa::PA8, Output, OpenDrain},
      prelude::*,
};

#[cfg(any(feature = "stm32f4xx", feature = "stm32h7xx"))] 
pub type TxType = Tx<USART1>;


pub type OpenDrainType = PA8<Output<OpenDrain>>;

//pub fn setup() ->  (OpenDrainType, I2cType, LedType, TxType, impl DelayNs, Clocks) {    
//    setup_from_dp(Peripherals::take().unwrap())
//}




//#[cfg(feature = "stm32f0xx")]
//use stm32f0xx_hal::{};

#[cfg(feature = "stm32f0xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32f0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {    
   let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
   let gpioa = dp.GPIOA.split(&mut rcc);

   let mut pin = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));
   pin.set_high().ok();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc),  &mut rcc);

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

   (pin, i2c, led, tx, delay, clocks)
}


#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    rcc::Clocks,
    serial::{Config},
};

#[cfg(feature = "stm32f1xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
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
    let i2c = setup_i2c1(dp.I2C1, gpiob, &mut afio, &clocks);

    let mut led = setup_led(dp.GPIOC.split()); 
    led.off();

    //let delay = DelayType{};
    let delay = dp.TIM2.delay_us(&clocks);

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

    (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    gpio::{gpioa::{PA9}, PushPull, AF7 },
};

#[cfg(feature = "stm32f3xx")]
pub type TxType = Tx<USART1, PA9<AF7<PushPull>>>;
//pub type TxType = Tx<USART1, impl TxPin<USART1>>;  // impl is unstable in type alias
// See  https://github.com/stm32-rs/stm32f3xx-hal/issues/288
//   regarding why it is necessary to specify the concrete pin here.

#[cfg(feature = "stm32f3xx")]
pub fn setup_from_dp(dp: Peripherals) -> (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut flash = dp.FLASH.constrain();
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze(&mut flash.acr);

   let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
   let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc.ahb);
   let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);

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

   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    rcc::{Clocks, RccExt},
    timer::TimerExt,
    gpio::GpioExt,
    serial::{config::Config},
};


#[cfg(feature = "stm32f4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

   let mut led = setup_led(dp.GPIOC.split()); 
   led.off();

   let delay = dp.TIM5.delay(&clocks);

   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   let (tx, _rx) = Serial::new(
       dp.USART1,
       (tx, rx),
       Config::default().baudrate(115200.bps()),
       &clocks,
   )
   .unwrap()
   .split();

   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    pac,
    serial::{Config, Oversampling, DataBits, Parity},
};

#[cfg(feature = "stm32f7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut dht   = gpioa .pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

   let mut led = setup_led(dp.GPIOC.split());
   led.off();

   let delay = dp.TIM2.delay_us(&clocks);

   let (tx, _rx) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9   for console
            gpioa.pa10.into_alternate(),
        ), //rx pa10  for console
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

  (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    serial::{FullConfig},
};

#[cfg(feature = "stm32g0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc);

   let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let tx = gpioa.pa9; 
   let rx = gpioa.pa10;
   let (tx, _rx) = dp.USART1.usart((tx, rx), FullConfig::default(), &mut rcc).unwrap().split();

   let delay = dp.TIM2.delay(&mut rcc);

   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    rcc::{Clocks},
    time::ExtU32,
    timer::Timer,
    delay::DelayFromCountDownTimer,
    gpio::{ Alternate, gpioa::{ PA9} },
    serial::{FullConfig, NoDMA},
};


#[cfg(feature = "stm32g4xx")]
pub type TxType = Tx<USART1, PA9<Alternate<7_u8>>, NoDMA >;
//pub type TxType = Tx<USART1, PA9<Output<PushPull>>, NoDMA >;

#[cfg(feature = "stm32g4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().unwrap(); // Pull high to avoid confusing the sensor when initializing.

   let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let clocks = rcc.clocks;  // not sure if this is right
   
   let timer2 = Timer::new(dp.TIM3, &clocks);
   let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));
   //let timer2 = Timer::new(dp.TIM2, &clocks);
   //let mut delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));


   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   //let (tx, _rx) = Serial::new(dp.USART1,(tx, rx),...  would be nice
   let (tx, _rx) = dp.USART1.usart(tx, rx, FullConfig::default().baudrate(115200.bps()),
         &mut rcc).unwrap().split();

   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32h7xx")]
pub use stm32h7xx_hal::rcc::CoreClocks as Clocks;

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32h7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, impl DelayNs, Clocks) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
   let i2cx = ccdr.peripheral.I2C1;  //.I2C4;

   let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
   let mut led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
   led.off();

   // CountDownTimer not supported by embedded-hal 1.0.0 ??
   let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   let (tx, _rx) = dp.USART1.serial(
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


   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Serial1Ext, },
};

#[cfg(feature = "stm32l0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
   //let clocks = rcc.clocks;

   let gpioa = dp.GPIOA.split(&mut rcc);
   let (tx, _rx) = dp.USART1.usart(
        gpioa.pa9,
        gpioa.pa10,
        Config::default().baudrate(115200.Bd()),
        &mut rcc,
    )
    .unwrap()
    .split();

   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let mut led = setup_led(dp.GPIOC.split(&mut rcc));
   led.off();
   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);

   let delay = DelayType{};
   //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    rcc::Config as rccConfig,
    serial::{Config, SerialExt, },
    //serial::{Config, Rx, Serial1Ext, Serial2Ext, Serial4Ext, Tx},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rccConfig::hsi());

   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut pin = gpioa.pa8.into_open_drain_output();
   pin.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let mut led = setup_led(dp.GPIOC.split(&mut rcc).pc9);
   led.off();

   let delay = DelayType{};

   let (tx, _rx) = dp
       .USART1
       .usart(
           (gpioa.pa9, gpioa.pa10),
           Config::default().baudrate(115200.bps()),
           &mut rcc,
       )
       .unwrap()
       .split();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);

   (pin, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    serial::{Config as serialConfig, },
};

#[cfg(feature = "stm32l4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {
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
   let mut pin = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   pin.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
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

   (pin, i2c, led, tx, delay, clocks)
}

