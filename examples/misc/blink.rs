//! Blink  onboard LED if one is available, or PC13 otherwise.
//! compare  blink3 example and stm32f1xx_hal example blinky.rs.
//! Also see blink_impl.rs  for the same example using implied traits in the setup() argument.
//!
//! stm32f1xx below uses PC13  which is onboard green LED on (many?) bluepill boards.
//! stm32f3xx below uses PE15  which is onboard green LD6 (West) LED on STM32F303 Discovery kit.
//! stm32f4xx below uses PC13  which is onboard blue C13  LED on some STM32F411CEU6 blackpill boards,
//!  another option would be PA5  which is onboard green LD2 LED on STM32F411RET6 Nucleo-64 board.
//! stm32l1xx below uses PB6   On some STM32L1.. Discovery boards there are onboard LD3 and LD4 LEDs on PB7
//!                            and PB6 but mine are defective and so tested with off board LED on PB6.
//!                            Heltec-lora-node151 tested with off board LED on PB6.
//!
//! Note of Interest:  board wiring can have the LED cathode connected to the GPIO pin and anode to Vcc,
//! so pin low is a sink and allows current flow. Alternate wiring has the GPIO pin as source.
//! Thus set_high() for the pin turns the LED off in one case (eg. bluepill and blackpill boards)
//! and on in the other (eg. Discovery & Nucleo-64 boards) with set_low() doing the opposite in each case.
//! To achieve generic code for turning the LED on and off an LED trait is defined, with different boards
//! having different use of set_high() and set_low() in their implemantations of set_on() and set_off().

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

// use panic_halt as _;  // put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort; // may still require nightly?
// use panic_itm;   // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

use embedded_hal::digital::v2::OutputPin;

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.
// 1. Get device specific peripherals
// 2. Take ownership of the raw rcc (Reset and Clock Control) device and convert to  HAL structs
// 3. Configure gpio pin as a push-pull output.
// 4. See Note of Interest above.

#[cfg(feature = "stm32f0xx")] //  eg  stm32f303x4
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (PC13<Output<PushPull>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

    let gpioc = p.GPIOC.split(&mut rcc);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    // led on pc13 with on/off
    let led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

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
fn setup() -> (PC13<Output<PushPull>>, Delay) {
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
fn setup() -> (PE15<Output<PushPull>>, Delay) {
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

    // the hal delay function paniced if the delay time was set at 2098ms or above.
    // see https://github.com/stm32-rs/stm32f3xx-hal/issues/203
    // delay fixed https://github.com/stm32-rs/stm32f3xx-hal/pull/208};

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
fn setup() -> (PC13<Output<PushPull>>, Delay) {
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

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (PC13<Output<PushPull>>, Delay) {
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
    //hal::digital::v2::OutputPin,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (PC13<Output<PushPull>>, Delay) {
    // see https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs
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
fn setup() -> (PC13<Output<PushPull>>, Delay) {
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
fn setup() -> (PB6<Output<PushPull>>, Delay) {
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
fn setup() -> (PC13<Output<PushPull>>, Delay) {
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

pub trait LED {
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();
}

#[entry]
fn main() -> ! {
    let (mut led, mut delay) = setup();

    let on: u32 = 1000;
    let off: u32 = 3000;

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        let _r = led.on();
        delay.delay_ms(on);
        let _r = led.off();
        delay.delay_ms(off);
    }
}
