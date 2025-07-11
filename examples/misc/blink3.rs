//! Blinks off-board LEDs attached to  pb 13,14,15.
//! compare example blink.rs and  stm32f1xx_hal example blinky.rs.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use embedded_hal::digital::OutputPin;

pub trait LED: OutputPin { 
    // Note: This is different from the onboard LED setting!
    // All leds are wired with pin as source, cathode connect to ground though a resistor.
    // The default is set_high() for on!

    fn on(&mut self) -> () {
        self.set_high().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_low().unwrap()
    }
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32f0xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32f0xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32f0xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

    let gpiob = p.GPIOB.split(&mut rcc);

    // Next is following examples in https://github.com/stm32-rs/stm32f0xx-hal/
    // I do not understand the logic and advantage / disadvantage of this  relative
    // to the setup for other hals.

    let (led1, led2, led3) = cortex_m::interrupt::free(move |cs| {
        (
            gpiob.pb13.into_push_pull_output(cs), // led on pb13
            gpiob.pb14.into_push_pull_output(cs), // led on pb14
            gpiob.pb15.into_push_pull_output(cs), // led on pb15
        )
    });

    // return (led1, led2, led3, delay)
    (led1, led2, led3, Delay::new(cp.SYST, &rcc))
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    timer::SysDelay as Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32f1xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32f1xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32f1xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpiob = p.GPIOB.split();

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(&mut gpiob.crh), // led on pb13
        gpiob.pb14.into_push_pull_output(&mut gpiob.crh), // led on pb14
        gpiob.pb15.into_push_pull_output(&mut gpiob.crh), // led on pb15
        cp.SYST.delay(&clocks),
    )
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32f3xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32f3xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32f3xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

    // return (led1, led2, led3, delay)
    (
        gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper), //led on pb13
        gpiob
            .pb14
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper), //led on pb14
        gpiob
            .pb15
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper), //led on pb15
        Delay::new(cp.SYST, clocks),
    )
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    timer::SysDelay as Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    rcc::Config,
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32f4xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32f4xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32f4xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let delay = cp.SYST.delay(&mut rcc.clocks);

    let gpiob = p.GPIOB.split(&mut rcc);

    rcc.freeze(
        Config::hsi()
           .hclk(48.MHz())
           .sysclk(48.MHz())
           .pclk1(24.MHz())
           .pclk2(24.MHz()),
    );

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        delay,
    )
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    timer::SysDelay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32f7xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32f7xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32f7xx")]
fn setup() -> (impl LED, impl LED, impl LED, SysDelay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let gpiob = p.GPIOB.split();

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        cp.SYST.delay(&clocks),
     )
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    timer::delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{Peripherals, TIM2, },
    prelude::*,
};

#[cfg(feature = "stm32g0xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32g0xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32g0xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32g0xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay<TIM2>) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpiob = dp.GPIOB.split(&mut rcc);

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        dp.TIM2.delay(&mut rcc)
    )
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    //delay::Delay,  //SysDelay
    time::{ExtU32},
    timer::{Timer},
    delay::DelayFromCountDownTimer,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32g4xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32g4xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32g4xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32g4xx")]
fn setup() -> (impl LED, impl LED, impl LED, impl DelayNs) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.clocks;  

    let gpiob = dp.GPIOB.split(&mut rcc);

    let timerx = Timer::new(dp.TIM2, &clocks);
    let delay = DelayFromCountDownTimer::new(timerx.start_count_down(100.millis()));

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        delay,
    )
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    //hal::digital::v2::impl LED, impl LED, impl LED, Delay,,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::delay::DelayNs;

#[cfg(feature = "stm32h7xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32h7xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32h7xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32h7xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    // see https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &p.SYSCFG);
    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        Delay::new(cp.SYST, ccdr.clocks),
    )
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32l0xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32l0xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32l0xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let gpiob = p.GPIOB.split(&mut rcc);

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        cp.SYST.delay(rcc.clocks),
    )
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    stm32::{CorePeripherals, Peripherals},
};

#[cfg(feature = "stm32l1xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32l1xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32l1xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32l1xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());

    let gpiob = p.GPIOB.split(&mut rcc);

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        Delay::new(cp.SYST, rcc.clocks),    // delay
    )
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
impl LED for PB13<Output<PushPull>> {}
#[cfg(feature = "stm32l4xx")]
impl LED for PB14<Output<PushPull>> {}
#[cfg(feature = "stm32l4xx")]
impl LED for PB15<Output<PushPull>> {}

#[cfg(feature = "stm32l4xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // return (led1, led2, led3, delay)
    (
        gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper), // led on pb13
        gpiob
            .pb14
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper), // led on pb14
        gpiob
            .pb15
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper), // led on pb15
        Delay::new(cp.SYST, clocks),
    )
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (mut led1, mut led2, mut led3, mut delay) = setup();

    let on: u32 = 1000; // milli-seconds (MPUs adjusted using mhz in setup)
    let off: u32 = 3000;

    // Wait for the timer to trigger an update and change the state of the LEDs
    loop {
        let _r = led1.on();
        let _r = led2.on();
        let _r = led3.on();
        delay.delay_ms(on);

        let _r = led1.off();
        let _r = led2.off();
        let _r = led3.off();
        delay.delay_ms(off);
    }
}
