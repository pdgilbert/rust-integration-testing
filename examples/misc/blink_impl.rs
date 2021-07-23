//! Blink  onboard LED if one is available, or PC13 otherwise.
//! See blink.rs  example for more details.
//! Relative to blink.rs this file uses trait LED with trait bounds and
//!    fn setup() -> (impl LED, Delay)
//! so setups all have similar and simpler signature.

// Example use of impl trait: If the LED  output pin is PC13 then
//      fn setup() -> (PC13<Output<PushPull>>, Delay) {
// is changed to
//      fn setup() -> (impl LED, Delay) {

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

//use embedded_hal::digital::v2::OutputPin;

pub trait LED {
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off()
    }
    fn blink_ok(&mut self, delay: &mut Delay) -> () {
        let dot: u16 = 5;
        let dash: u16 = 100;
        let spc: u16 = 300;
        let space: u16 = 800;
        let end: u16 = 1500;

        // dash-dash-dash
        self.blink(dash, delay);
        delay.delay_ms(spc);
        self.blink(dash, delay);
        delay.delay_ms(spc);
        self.blink(dash, delay);
        delay.delay_ms(space);

        // dash-dot-dash
        self.blink(dash, delay);
        delay.delay_ms(spc);
        self.blink(dot, delay);
        delay.delay_ms(spc);
        self.blink(dash, delay);
        delay.delay_ms(end);
    }
}

#[cfg(feature = "stm32f0xx")] //  eg  stm32f303x4
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

    let gpioc = p.GPIOC.split(&mut rcc);

    // led on pc13 with on/off
    let led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // return tuple  (led, delay)
    (led, Delay::new(cp.SYST, &rcc))
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(&mut gpioc.crh), // led on pc13 with on/off
        Delay::new(cp.SYST, clocks),
    )
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioe::PE15, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb);

    impl LED for PE15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpioe
            .pe15
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), // led on pe15 with on/off
        Delay::new(cp.SYST, clocks),
    )
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    //gpio::{gpioa::PA5, Output, PushPull,},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc
        .cfgr
        .hclk(48.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .pclk2(24.mhz())
        .freeze();

    let gpioc = p.GPIOC.split();

    // Note that blackpill with stm32f411 and nucleo-64 with stm32f411 have onboard led wired
    // differently, so this is reversed (in addition to PA5 vs PC13).

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        Delay::new(cp.SYST, &clocks),
    )
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (impl LED, Delay) {
    // fn setup() -> (PC13<Output<PushPull>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.sysclk(216.mhz()).freeze();

    let gpioc = p.GPIOC.split();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        Delay::new(cp.SYST, clocks),
    )
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &p.SYSCFG); // calibrate for correct blink rate
    let gpioc = p.GPIOC.split(ccdr.peripheral.GPIOC);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        Delay::new(cp.SYST, ccdr.clocks),   //SysTick: System Timer  delay
    )
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let gpioc = p.GPIOC.split(&mut rcc);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        Delay::new(cp.SYST, rcc.clocks),
    )
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpiob::PB6, Output, PushPull},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    stm32::{CorePeripherals, Peripherals},
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (impl LED, Delay) {
    //fn setup() -> (PB6<Output<PushPull>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.freeze(rcc::Config::hsi());

    let gpiob = p.GPIOB.split();

    impl LED for PB6<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpiob.pb6.into_push_pull_output(), // led on pb6 with on/off
        cp.SYST.delay(rcc.clocks),
    )
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioc = p.GPIOC.split(&mut rcc.ahb2);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // return tuple  (led, delay)
    (
        gpioc
            .pc13
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper), // led on pc13 with on/off
        Delay::new(cp.SYST, clocks),
    )
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (mut led, mut delay) = setup();

    led.blink_ok(&mut delay); // blink OK to indicate setup complete and main started.

    let on: u16 = 10; // on for 10 ms
    let off: u16 = 3000; //off for  3 s

    // blink LED and sleep
    loop {
        led.blink(on, &mut delay);
        delay.delay_ms(off);
    }
}
