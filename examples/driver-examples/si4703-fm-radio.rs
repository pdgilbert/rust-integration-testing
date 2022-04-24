// Code here is based on https://github.com/eldruin/driver-examples/

//! Seek an FM radio channel when pressing two buttons "Seek down" / "Seek up"
//! using an Si4703 FM radio receiver (turner).
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/si4703-fm-radio-receiver-driver-in-rust/
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1.
//!
//! ```
//! BP    <-> Si4703
//! GND   <-> GND
//! +3.3V <-> VCC
//! PB8   <-> SCLK
//! PB9   <-> SDIO
//! PB7   <-> RST
//! PB6   <-> GPIO2
//! PB10             <-> Seek up button   <-> +3.3V
//! PB11             <-> Seek down button <-> +3.3V
//! ```
//!
//! Run with:
//! `cargo embed --example si4703-fm-radio-bp`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nb::block;
//use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use si4703::{
    reset_and_select_i2c_method1 as reset_si4703, ChannelSpacing, DeEmphasis, SeekDirection,
    SeekMode, Si4703, Volume,
};

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

//use rust_integration_testing_of_examples::i2c_led_delay::{setup_i2c, setup_led, LED, DelayType};
use rust_integration_testing_of_examples::led::{setup_led, LED};
//use rust_integration_testing_of_examples::i2c::{setup_i2c1};
use rust_integration_testing_of_examples::delay::{DelayType};

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{Input, PullDown, PullUp,
        gpiob::{PB10, PB11, PB6},
    },
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
    let mut delay = Delay::new(cp.SYST, &rcc);

    let gpiob = dp.GPIOB.split(&mut rcc);

    let (scl, mut sda, mut rst, stcint, seekup, seekdown) =
        cortex_m::interrupt::free(move |cs| {
            (
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

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), &mut rcc);
    let led = setup_led(dp.GPIOC.split(&mut rcc));

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
    //timer::Delay,
    //timer::SysDelay as Delay,
    gpio::{
        gpiob::{PB10, PB11, PB6},
        Input, PullDown, PullUp,
    },
    i2c::{BlockingI2c, DutyCycle, Mode, Pins},
    pac::{Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    //let mut delay = cp.SYST.delay(&clocks);
    let mut delay = dp.TIM2.delay_us(&clocks);

    let mut afio = dp.AFIO.constrain();

    let led = setup_led(dp.GPIOC.split());

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
            frequency: 400_000.Hz(),
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
    gpio::{Input,
        gpiob::{PB10, PB11, PB6},
    },
    i2c::{I2c, SclPin, SdaPin},
    pac::{Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input>,
) {
    //let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    //let mut delay = Delay::new(cp.SYST, clocks);
    let mut delay = DelayType{};

    let led = setup_led(dp.GPIOE.split(&mut rcc.ahb));

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let mut sda = gpiob
        .pb9
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut rst = gpiob
        .pb7
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let stcint = gpiob
        .pb6
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

    let scl = gpiob
        .pb8
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);

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
    //timer::SysDelay as Delay,
    gpio::{Input,
           gpiob::{PB10, PB11, PB6},        
    },
    i2c::{I2c, Pins},
    pac::{Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input>,
) {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    //let mut delay = Delay::new(cp.SYST, &clocks);
    //let mut delay = cp.SYST.delay(&clocks);
    let mut delay = dp.TIM2.delay_us(&clocks);

    let gpiob = dp.GPIOB.split(); // for i2c

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate().set_open_drain();
    let stcint = gpiob.pb6.into_pull_up_input();

    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);
    let led = setup_led(dp.GPIOC.split());

    let buttons: SeekPins<PB10<Input>, PB11<Input>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input>, PB11<Input>> {
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
    gpio::{Input, PullDown, PullUp,
           gpiob::{PB10, PB11, PB6},
    },
    i2c::{BlockingI2c, Mode, PinScl, PinSda},
    pac::{Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    //let mut delay = Delay::new(cp.SYST, clocks);
    let mut delay = DelayType{};

    let led = setup_led(dp.GPIOC.split());

    let gpiob = dp.GPIOB.split();
    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate_open_drain();
    let stcint = gpiob.pb6.into_pull_up_input();

    let scl = gpiob.pb8.into_alternate_open_drain();

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut rcc.apb1,
        1000,
    );

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

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    gpio::{Input,
        gpiob::{PB10, PB11, PB6},
    },
    i2c::I2c,
    pac::{Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (I2c<I2C1>, impl LED, DelayType, impl SEEK, PB6<Input>) {
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    //let mut delay = Delay::new(cp.SYST, clocks);
    let mut delay = DelayType{};

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));

    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_alternate().set_open_drain();
    let stcint = gpiob.pb6.into_pull_up_input();

    let scl = gpiob.pb8.into_alternate().set_open_drain(); // scl on PB8

    let i2c = dp
        .I2C1
        .i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &clocks);

    let buttons: SeekPins<PB10<Input>, PB11<Input>> = SeekPins {
        p_seekup: gpiob.pb10.into_pull_down_input(),
        p_seekdown: gpiob.pb11.into_pull_down_input(),
    };

    impl SEEK for SeekPins<PB10<Input>, PB11<Input>> {
        fn seekup(&mut self) -> bool {
            self.p_seekup.is_high()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high()
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

    let led = setup_led(p.GPIOC.split(&mut rcc));

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
    gpio::{Input, PullDown, PullUp,
           gpiob::{ PB10, PB11, PB6},
    },
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    stm32::{Peripherals, I2C1},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::{InputPin};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    I2c<I2C1, impl Pins<I2C1>>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    //let clocks = rcc.clocks;

    //let mut delay = Delay::new(cp.SYST, clocks);
    let mut delay = DelayType{};

    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut rst = gpiob.pb7.into_push_pull_output();
    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();
    let sda = sda.into_open_drain_output();
    let stcint = gpiob.pb6.into_pull_up_input();
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8

    let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);
    let led = setup_led(dp.GPIOC.split(&mut rcc).pc9);

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
    gpio::{Input, PullDown, PullUp,
           gpiob::{PB10, PB11, PB6},
    },
    i2c::{Config as i2cConfig, I2c, SclPin, SdaPin},
    pac::{Peripherals, I2C1},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    DelayType,
    impl SEEK,
    PB6<Input<PullUp>>,
) {
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    //let mut delay = Delay::new(cp.SYST, clocks);
    let mut delay = DelayType{};

    let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);

    let mut sda = gpiob
        .pb9
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let mut rst = gpiob
        .pb7
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();

    let sda = sda.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);

    let stcint = gpiob
        .pb6
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

    //this should be simpler
    let mut scl =
        gpiob
            .pb8
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        i2cConfig::new(400.kHz(), clocks),
        &mut rcc.apb1r1,
    );

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
            self.p_seekup.is_high()
        }
        fn seekdown(&mut self) -> bool {
            self.p_seekdown.is_high()
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

    //    let mut rst = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
    //    let stcint = gpiob.pb6.into_pull_up_input(&mut gpiob.crl);
    //    let seekdown = gpiob.pb11.into_pull_down_input(&mut gpiob.crh);
    //    let seekup = gpiob.pb10.into_pull_down_input(&mut gpiob.crh);
    //    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();

    let manager = shared_bus::BusManagerSimple::new(i2c);

    let mut radio = Si4703::new(manager.acquire_i2c());
    radio.enable_oscillator().unwrap();
    delay.delay_ms(500_u16);
    radio.enable().unwrap();
    delay.delay_ms(110_u16);

    radio.set_volume(Volume::Dbfsm28).unwrap();
    radio.set_deemphasis(DeEmphasis::Us50).unwrap();
    radio.set_channel_spacing(ChannelSpacing::Khz100).unwrap();
    radio.unmute().unwrap();
    loop {
        // Blink LED 0 every time a new seek is started
        // to check that everything is actually running.
        led.blink(50_u16, &mut delay);

        let should_seek_down = buttons.seekdown();
        let should_seek_up = buttons.seekup();
        if should_seek_down || should_seek_up {
            let direction = if should_seek_down {
                SeekDirection::Down
            } else {
                SeekDirection::Up
            };

            block!(radio.seek_with_stc_int_pin(SeekMode::Wrap, direction, &stcint)).unwrap();
        }
    }
}
