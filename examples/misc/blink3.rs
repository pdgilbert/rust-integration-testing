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

pub trait LED {
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();
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
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

    let gpiob = p.GPIOB.split(&mut rcc);

    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

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
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpiob = p.GPIOB.split();

    //this would work for delay on bluepill but not others
    //use stm32f1xx_hal::timer::Timer;
    // trigger an update every second
    // let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());
    // /block!(timer.wait()).unwrap();

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(&mut gpiob.crh), // led on pb13
        gpiob.pb14.into_push_pull_output(&mut gpiob.crh), // led on pb14
        gpiob.pb15.into_push_pull_output(&mut gpiob.crh), // led on pb15
        Delay::new(cp.SYST, clocks),
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
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

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
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
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

    let gpiob = p.GPIOB.split();

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        Delay::new(cp.SYST, &clocks),
    )
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Output, PushPull,
    },
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let gpiob = p.GPIOB.split();

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

    // return (led1, led2, led3, delay)
    (
        gpiob.pb13.into_push_pull_output(), // led on pb13
        gpiob.pb14.into_push_pull_output(), // led on pb14
        gpiob.pb15.into_push_pull_output(), // led on pb15
        Delay::new(cp.SYST, clocks),
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
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    // see https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &p.SYSCFG);
    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

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
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let gpiob = p.GPIOB.split(&mut rcc);

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

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
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.freeze(rcc::Config::hsi());

    let gpiob = p.GPIOB.split();

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

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
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l4xx")]
fn setup() -> (impl LED, impl LED, impl LED, Delay) {
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

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // all leds wire with pin as source, cathode connect to ground though a resistor.
    impl LED for PB13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB14<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    impl LED for PB15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

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
