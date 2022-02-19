//! Note that led and i2c pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type};



// setup() does all  HAL/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
pub fn setup() -> ( I2c1Type, LedType, Delay) {
    let mut dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);
    let led = setup_led(dp.GPIOC.split(&mut rcc));
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &rcc);

    (i2c, led, delay)
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals,},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
pub fn setup() -> (I2c1Type, LedType, Delay) {
    //            (BlockingI2c<I2C1, impl Pins<I2C1>>, impl LED, , Delay)  works too 
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &mut dp.AFIO.constrain(), &clocks);
    let led = setup_led(dp.GPIOC.split());
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &clocks);

    (i2c, led, delay)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals,},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
pub fn setup() -> (I2c1Type, LedType,Delay) {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    
    let gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);
    let led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    (i2c, led, delay)
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    timer::SysDelay,
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
pub type DelayType = SysDelay;

#[cfg(feature = "stm32f4xx")]
pub fn setup() -> (I2c<I2C1, impl Pins<I2C1>>, LedType, DelayType) {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)
    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

    let led = setup_led(dp.GPIOC.split());
    //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &clocks);
    let cp = CorePeripherals::take().unwrap();
    let delay = cp.SYST.delay(&clocks);

    (i2c, led, delay)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals, },
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
pub fn setup() -> (I2c1Type, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

    let led = setup_led(dp.GPIOC.split());
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    (i2c, led, delay)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
pub fn setup() -> (I2c<I2C1>, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let i2cx = ccdr.peripheral.I2C1;

    let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
    let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    (i2c, led, delay)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{OpenDrain, Output, PushPull,
           gpiob::{PB8, PB9, Parts as PartsB},
           gpioc::{PC13, Parts as PartsC},
    },
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
pub fn setup() -> (I2c1Type, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let clocks = rcc.clocks;

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), &mut rcc, &clocks);
    let led = setup_led(dp.GPIOC.split(&mut rcc));
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    (i2c, led, delay)
}


#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    stm32::{CorePeripherals, Peripherals, I2C1},
};

#[cfg(feature = "stm32l1xx")]
pub fn setup() -> (I2c<I2C1, impl Pins<I2C1>>, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    let clocks = rcc.clocks;

    let gpiob = dp.GPIOB.split(&mut rcc);

    // Note this example is especially tricky:
    //   The onboard led is on PB6 and i2c also uses gpiob  so there is a problem
    //   with gpiob being moved by one and then not available for the other.

// setup_i2c1 NOT WORKING
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);
//    let i2c = setup_i2c1(dp.I2C1, &mut gpiob, rcc);

    let led = setup_led(gpiob.pb6);
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    (i2c, led, delay)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
pub fn setup() -> (I2c1Type, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80.mhz()).pclk1(80.mhz()).pclk2(80.mhz()).freeze(&mut flash.acr, &mut pwr);

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
    let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    (i2c, led, delay)
}

// End of HAL/MCU specific setup.
