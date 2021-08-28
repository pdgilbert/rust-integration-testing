// see https://github.com/eldruin/driver-examples/issues/2
// and si4703-fm-radio-display.rs is more developed but also not working

//! Seek an FM radio channel when pressing two buttons "Seek down" / "Seek up"
//! using an Si4703 FM radio receiver (turner) and display the channel
//! frequency in an SSD1306 OLED display.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/si4703-fm-radio-receiver-driver-in-rust/
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1.
//!
//! ```
//! BP    <-> Si4703 <-> Display
//! GND   <-> GND    <-> GND
//! +3.3V <-> VCC    <-> VCC
//! PB8   <-> SCLK   <-> SCL
//! PB9   <-> SDIO   <-> SDA
//! PB7   <-> RST
//! PB6   <-> GPIO2
//! PB10                        <-> Seek up button   <-> +3.3V
//! PB11                        <-> Seek down button <-> +3.3V
//! ```
//!
//! Run with:
//! `cargo embed --example si4703-fm-radio-bp`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use si4703::{
    reset_and_select_i2c_method1 as reset_si4703, ChannelSpacing, DeEmphasis, ErrorWithPin,
    SeekDirection, SeekMode, Si4703, Volume,
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

pub trait LED {
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // implementation should deal with this difference
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off();
        delay.delay_ms(time); // consider delay.delay_ms(500u16);
    }
}

pub struct SeekPins<T, U> {
    p_seekup: T,
    p_seekdown: U,
    //stcint:   V,
}

pub trait SEEK {
    fn seekup(&mut self) -> bool;
    fn seekdown(&mut self) -> bool;
    //fn stcint(&mut self) -> ;
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
    let mut delay = Delay::new(cp.SYST, &rcc);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let (led, scl, mut sda, mut rst, stcint, seekup, seekdown) =
        cortex_m::interrupt::free(move |cs| {
            (
                gpioc.pc13.into_push_pull_output(cs),
                gpiob.pb8.into_alternate_af1(cs),
                //gpiob.pb9.into_alternate_af1(cs),      //for i2c
                gpiob.pb9.into_push_pull_output(cs), //for reset
                gpiob.pb7.into_push_pull_output(cs),
                gpiob.pb6.into_pull_up_input(cs),
                gpiob.pb10.into_pull_down_input(cs),
                gpiob.pb11.into_pull_down_input(cs),
            )
        });

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = cortex_m::interrupt::free(move |cs| sda.into_alternate_af1(cs));

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), &mut rcc);

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: seekup,
        p_seekdown: seekdown,
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::{BlockingI2c, DutyCycle, Mode, Pins},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut afio = dp.AFIO.constrain();

    let mut gpioc = dp.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let mut sda = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
    let mut rst = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate_open_drain(&mut gpiob.crh);
    let stcint = gpiob.pb6.into_pull_up_input(&mut gpiob.crl);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(&mut gpiob.crh),
        p_seekdown: gpiob.pb11.into_pull_down_input(&mut gpiob.crh),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioe::PE9,
        Input, Output, PushPull,
    },
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let led = gpioe
        .pe9
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    impl LED for PE9<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let mut sda = gpiob
        .pb9
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut rst = gpiob
        .pb7
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let stcint = gpiob
        .pb6
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

    let scl = gpiob
        .pb8
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 400_000.Hz(), clocks, &mut rcc.apb1);

    let buttons: SeekPins<PB10<Input>, PB11<Input>> = SeekPins {
        p_seekup: gpiob
            .pb10
            .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr),
        p_seekdown: gpiob
            .pb11
            .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr),
    };

    impl SEEK for SeekPins<PB10<Input>, PB11<Input>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut delay = Delay::new(cp.SYST, &clocks);

    // led
    let gpioc = dp.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let gpiob = dp.GPIOB.split(); // for i2c

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate().set_open_drain();
    let stcint = gpiob.pb6.into_pull_up_input();

    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.khz(), clocks);

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::{BlockingI2c, Mode, PinScl, PinSda},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut delay = Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate_af4().set_open_drain();
    let stcint = gpiob.pb6.into_pull_up_input();

    let scl = gpiob.pb8.into_alternate_af4().set_open_drain();

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        clocks,
        &mut rcc.apb1,
        1000,
    );

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::{InputPin, OutputPin};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (I2c<I2C1>, impl LED, Delay, impl SEEK, PB6<Input<PullUp>>) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    let mut delay = Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate_af4().set_open_drain();
    let stcint = gpiob.pb6.into_pull_up_input();

    let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8

    let i2c = dp
        .I2C1
        .i2c((scl, sda), 400.khz(), ccdr.peripheral.I2C1, &clocks);

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpiob::{PB8, PB9},
        gpioc::PC13,
        Input, OpenDrain, Output, PullDown, PullUp, PushPull,
    },
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    I2c<I2C1, PB9<Output<OpenDrain>>, PB8<Output<OpenDrain>>>,
    //I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let clocks = rcc.clocks;
    let mut delay = Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    // led
    let led = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_open_drain_output();
    let stcint = gpiob.pb6.into_pull_up_input();

    let scl = gpiob.pb8.into_open_drain_output();

    let i2c = dp.I2C1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    stm32::{CorePeripherals, Peripherals, I2C1},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::{InputPin, OutputPin};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    let clocks = rcc.clocks;
    let mut delay = Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_open_drain_output();
    let stcint = gpiob.pb6.into_pull_up_input();

    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8

    let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        gpioc::PC13,
        Input, Output, PullDown, PullUp, PushPull,
    },
    i2c::{I2c, SclPin, SdaPin, Config as i2cConfig},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
use embedded_hal::digital::v2::{InputPin, OutputPin};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    Delay,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);

    let led = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper); // led on pc13

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);

    let mut sda = gpiob
        .pb9
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let mut rst = gpiob
        .pb7
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();

    let sda = sda
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
        .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let stcint = gpiob
        .pb6
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

    //this should be simpler
    let mut scl = gpiob
        .pb8
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    scl.internal_pull_up(&mut gpiob.pupdr, true);
    let scl = scl.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), i2cConfig::new(400.khz(), clocks), &mut rcc.apb1r1);   

    let buttons: SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> = SeekPins {
        p_seekup: gpiob
            .pb10
            .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr),
        p_seekdown: gpiob
            .pb11
            .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr),
    };

    impl SEEK for SeekPins<PB10<Input<PullDown>>, PB11<Input<PullDown>>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high().unwrap()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high().unwrap()
        }
    }

    (i2c, led, delay, buttons, stcint)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Si4703 example");

    let (i2c, mut led, mut delay, mut buttons, stcint) = setup();

    let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut radio = Si4703::new(manager.acquire());
    radio.enable_oscillator().unwrap();
    delay.delay_ms(500_u16);
    radio.enable().unwrap();
    delay.delay_ms(110_u16);

    radio.set_volume(Volume::Dbfsm28).unwrap();
    radio.set_deemphasis(DeEmphasis::Us50).unwrap();
    radio.set_channel_spacing(ChannelSpacing::Khz100).unwrap();
    radio.unmute().unwrap();

    let mut buffer: heapless::String<64> = heapless::String::new();
    loop {
        // Blink LED 0 every time a new seek is started
        // to check that everything is actually running.
        led.blink(50_u16, &mut delay);

        let should_seek_down = buttons.seekdown();
        let should_seek_up = buttons.seekup();
        if should_seek_down || should_seek_up {
            buffer.clear();
            write!(buffer, "Seeking...").unwrap();

            display.clear();
            Text::new(&buffer, Point::zero(), text_style)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();
            let direction = if should_seek_down {
                SeekDirection::Down
            } else {
                SeekDirection::Up
            };

            buffer.clear();
            loop {
                match radio.seek_with_stc_int_pin(SeekMode::Wrap, direction, &stcint) {
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(ErrorWithPin::SeekFailed)) => {
                        write!(buffer, "Seek Failed!  ").unwrap();
                        break;
                    }
                    Err(_) => {
                        write!(buffer, "Error!     ").unwrap();
                        break;
                    }
                    Ok(_) => {
                        let channel = radio.channel().unwrap_or(-1.0);
                        write!(buffer, "Found {:1} MHz ", channel).unwrap();
                        break;
                    }
                }
            }
            display.clear();
            Text::new(&buffer, Point::zero(), text_style)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();
        }
    }
}
