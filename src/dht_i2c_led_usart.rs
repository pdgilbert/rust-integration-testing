
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
      pac::{Peripherals, CorePeripherals},
      gpio::{gpioa::PA8, Output, OpenDrain},
      prelude::*,
};

type DhtType = PA8<Output<OpenDrain>>;


pub fn setup() ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {    
    setup_from_dp(Peripherals::take().unwrap())
}




#[cfg(feature = "stm32f0xx")]
use stm32f0xx_hal::{
    //delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{USART1},
    prelude::*,
    serial::{Serial, Tx},
};

#[cfg(feature = "stm32f0xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32f0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {    
   let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
   let gpioa = dp.GPIOA.split(&mut rcc);

   let mut dht = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));
   dht.set_high().ok();

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

   (dht, i2c, led, tx, delay, clocks)
}


#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    device::USART1,
    gpio::{OpenDrain, Output,
        gpioa::{PA8},
    },
    prelude::*,
    serial::{Config, Serial, Tx},
};



#[cfg(feature = "stm32f1xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32f1xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
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
    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

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

    (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    gpio::{
        gpioa::{PA8, PA9},
        OpenDrain, Output, PushPull, AF7,
    },
    pac::{USART1},
    prelude::*,
    serial::{Serial, Tx},
};

#[cfg(feature = "stm32f3xx")]
pub type TxType = Tx<USART1, PA9<AF7<PushPull>>>;
//pub type TxType = Tx<USART1, impl TxPin<USART1>>;  // impl is unstable in type alias
// See  https://github.com/stm32-rs/stm32f3xx-hal/issues/288
//   regarding why it is necessary to specify the concrete pin here.

#[cfg(feature = "stm32f3xx")]
pub fn setup_from_dp(dp: Peripherals) -> (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut flash = dp.FLASH.constrain();
   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze(&mut flash.acr);

   let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
   let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

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

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32f4xx")]
pub use stm32f4xx_hal::rcc::Clocks;

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    rcc::{RccExt},
    timer::TimerExt,
    gpio::GpioExt,
    pac::USART1,
    serial::{config::Config, Serial, Tx},
};


#[cfg(feature = "stm32f4xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32f4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

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

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    gpio::{Output, OpenDrain,
        gpioa::PA8,
    },
    pac,
    prelude::*,
    serial::{Config, Oversampling, Serial, Tx, DataBits, Parity},
};

#[cfg(feature = "stm32f7xx")]
pub type TxType = Tx<pac::USART2>;

#[cfg(feature = "stm32f7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let gpioa = dp.GPIOA.split();
   let mut dht   = gpioa .pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let mut rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
   //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

   let mut led = setup_led(dp.GPIOC.split());
   led.off();

   let delay = dp.TIM2.delay_us(&clocks);

   let (tx, _rx) = Serial::new(
       dp.USART2,
       (
           gpioa.pa2.into_alternate(),
           gpioa.pa3.into_alternate(),
       ),
       &clocks,
       Config {
           baud_rate: 115200.bps(),
           data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
           parity: Parity::ParityEven,
           oversampling: Oversampling::By16,
           character_match: None,
           sysclock: false,
       },
   ).split();

  (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    gpio::{OpenDrain, Output,
           gpioa::PA8,
    },
    pac::USART1,
    prelude::*,
    serial::{FullConfig, Tx},
};

#[cfg(feature = "stm32g0xx")]
pub type TxType = Tx<USART1, FullConfig>;

#[cfg(feature = "stm32g0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(&mut rcc);

   let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let tx = gpioa.pa9; 
   let rx = gpioa.pa10;
   let (tx, _rx) = dp.USART1.usart((tx, rx), FullConfig::default(), &mut rcc).unwrap().split();

   let delay = dp.TIM2.delay(&mut rcc);

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    timer::Timer,
    delay::DelayFromCountDownTimer,
    gpio::{OpenDrain, Output, Alternate,
           gpioa::{PA8, PA9},
    },
    stm32::{USART1}, //I2C1
    prelude::*,
    serial::{FullConfig, Tx, NoDMA},
};


#[cfg(feature = "stm32g4xx")]
pub type TxType = Tx<USART1, PA9<Alternate<7_u8>>, NoDMA >;
//pub type TxType = Tx<USART1, PA9<Output<PushPull>>, NoDMA >;

#[cfg(feature = "stm32g4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.constrain();
   
   let gpioa = dp.GPIOA.split(&mut rcc);
   let gpiob = dp.GPIOB.split(&mut rcc);

   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high().unwrap(); // Pull high to avoid confusing the sensor when initializing.

   let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);

   let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
   led.off();

   let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
   let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.ms()));

   let tx = gpioa.pa9.into_alternate();
   let rx = gpioa.pa10.into_alternate();
   //let (tx, _rx) = Serial::new(dp.USART1,(tx, rx),...  would be nice
   let (tx, _rx) = dp.USART1.usart(tx, rx, FullConfig::default().baudrate(115200.bps()),
         &mut rcc).unwrap().split();

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32h7xx")]
pub use stm32h7xx_hal::rcc::CoreClocks as Clocks;

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    pac::{USART2,},
    serial::Tx,
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32h7xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32h7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
   let i2cx = ccdr.peripheral.I2C1;  //.I2C4;

   let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
   let mut led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
   led.off();

   // CountDownTimer not supported by embedded-hal 1.0.0 ??
   let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   let (tx, _rx) = dp
       .USART2
       .serial(
           (
               gpioa.pa2.into_alternate(),
               gpioa.pa3.into_alternate(),
           ),
           115200.bps(),
           ccdr.peripheral.USART2,
           &clocks,
       )
       .unwrap()
       .split();

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    gpio::{Output, OpenDrain, gpioa::PA8},
    //delay::Delay,
    pac::{USART1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Serial1Ext, Tx},
};

#[cfg(feature = "stm32l0xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32l0xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
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

   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

   let mut led = setup_led(dp.GPIOC.split(&mut rcc));
   led.off();
   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);

   let delay = DelayType{};
   //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    gpio::{OpenDrain, Output,
           gpioa::PA8,
    },
    prelude::*,
    rcc::Config as rccConfig,
    serial::{Config, SerialExt, Tx},
    stm32::{USART1},
    //serial::{Config, Rx, Serial1Ext, Serial2Ext, Serial4Ext, Tx},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32l1xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
   let mut rcc = dp.RCC.freeze(rccConfig::hsi());

   let gpioa = dp.GPIOA.split(&mut rcc);
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.

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

   (dht, i2c, led, tx, delay, clocks)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    gpio::{OpenDrain, Output,
        gpioa::PA8,
    },
    pac::{USART2},
    prelude::*,
    serial::{Config as serialConfig, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32l4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (DhtType, I2cType, LedType, TxType, Delay, Clocks) {
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
   let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
   let mut led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
   led.off();

   let delay = DelayType{};

   let (tx, _rx) = Serial::usart2(
       dp.USART2,
       (
           gpioa
               .pa2
               .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
           gpioa
               .pa3
               .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
       ),
       serialConfig::default().baudrate(115200.bps()),
       clocks,
       &mut rcc.apb1r1,
   )
   .split();

   (dht, i2c, led, tx, delay, clocks)
}

