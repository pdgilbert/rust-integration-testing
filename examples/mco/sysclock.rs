//  Set MCO pin to output sysclock

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use embedded_hal::delay::DelayNs;

use embedded_hal::digital::OutputPin;
use embedded_hal;

pub trait LED: OutputPin { 
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // A default of set_low() for on is defined here, but implementation should deal with a difference
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }
}

#[cfg(feature = "stm32f0xx")]
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

    // led on pc13 with on/off
    let led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

    // return tuple  (led, delay)
    (led, Delay::new(cp.SYST, &rcc))
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    rcc::Config,
    prelude::*,
};


#[cfg(feature = "stm32f1xx")]
fn setup() -> (PC13<Output<PushPull>>, impl DelayNs) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain().freeze(Config::hsi(), &mut p.FLASH.constrain().acr);
    //let mut rcc = p.RCC.constrain().freeze(
    //                       Config::hsi() .hclk(48.MHz()) .sysclk(48.MHz()) .pclk1(24.MHz()) .pclk2(24.MHz()),
    //                       &mut p.FLASH.constrain().acr );

    let mut gpioc = p.GPIOC.split(&mut rcc);

    // see examples in https://github.com/stm32-rs/stm32f1xx-hal/examples/
    //  for other (better) ways to do delay

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(&mut gpioc.crh), // led on pc13 with on/off
        cp.SYST.delay(&rcc.clocks)
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



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  blackpill stm32f411
use stm32f4xx_hal::{
    gpio::{gpioa::PA8, Output, PushPull,},
    pac::{CorePeripherals, Peripherals},
    rcc::Config,
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (PC13<Output<PushPull>>, impl DelayNs) {
    let dp = Peripherals::take().unwrap();

    // let clocks = rcc.cfgr.use_hse(25.MHz()).freeze();
    //let mut rcc = p.RCC.constrain().freeze(Config::hsi() .hclk(48.MHz()) .sysclk(48.MHz()) .pclk1(24.MHz()) .pclk2(24.MHz()) );
    let rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);

    rcc.cfgr().mco1(Mco1Clock::HSE); // Example: Set MCO1 to HSE // options HSI, HSE, PLL

    let mco = gpioa.pa8.into_alternate(); // Set PA8 as alternate function

    // Enable clock output on mco pin
    //dp.RCC.mco1.set_mco1(mco);

    (led, delay)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    timer::SysDelay,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (PC13<Output<PushPull>>, SysDelay) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let gpioc = dp.GPIOC.split();
    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        cp.SYST.delay(&clocks),
    )
}


#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    prelude::*,
    timer::delay::Delay,
    pac::{TIM2, Peripherals},
};

#[cfg(feature = "stm32g0xx")]
pub fn setup() -> (PC13<Output<PushPull>>, Delay<TIM2>) {//NOT SURE WHAT PIN THIS SHOULD BE
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioc = dp.GPIOC.split(&mut rcc);
    let led = gpioc.pc13.into_push_pull_output();
    let delay = dp.TIM2.delay(&mut rcc);
    
    (led, delay)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    time::{ExtU32},
    timer::{Timer},
    delay::DelayFromCountDownTimer,
    gpio::{gpioc::PC6, Output, PushPull},
    prelude::*,
    pac::{Peripherals},
    rcc::{Prescaler, MCOSrc},
};

// weact-stm32g474CEU6 has onboard led on PC6
#[cfg(feature = "stm32g4xx")]
    impl LED for PC6<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

#[cfg(feature = "stm32g4xx")]
pub fn setup() -> (impl LED, impl DelayNs) {
//pub fn setup() -> (PC6<Output<PushPull>>, impl DelayNs) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    
    let gpioc = dp.GPIOC.split(&mut rcc);
    let led = gpioc.pc6.into_push_pull_output();    
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    //  See  stm32g4xx-hal/src/rcc/clockout.rs
    // MCOSrc::{SysClk, HSI, HSE, PLL, LSI, LSE}
    //Prescaler::{NotDivided, Div2, Div4, Div8, Div16, Div32, Div64}
    //let pin = gpioa.pa8.into_alternate(); // Set PA8 as alternate function
    //let mco = pin.mco(MCOSrc::LSI, Prescaler::NotDivided, &mut rcc); 
    let mco = gpioa.pa8.mco(MCOSrc::SysClk, Prescaler::Div64, &mut rcc); // Also sets PA8 as alternate function

    // Enable clock output on mco pin
    mco.enable();
//SHOULD FREEZE HERE?

    (led, delay)
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
fn setup() -> (PC13<Output<PushPull>>,  impl DelayNs) {
    // see https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &p.SYSCFG); // calibrate for correct blink rate
    let gpioc = p.GPIOC.split(ccdr.peripheral.GPIOC);

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
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());

    let gpiob = p.GPIOB.split(& mut rcc);

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
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioc = p.GPIOC.split(&mut rcc.ahb2);

    // return tuple  (led, delay)
    (
        gpioc
            .pc13
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper), // led on pc13 with on/off
        Delay::new(cp.SYST, clocks),
    )
}


#[entry]
fn main() -> ! {
    let (mut led, mut delay) = setup();    
    //mco is already enabled in setup. Loop is just to indicate something is running.
    loop {
       led.on();
       delay.delay_ms(20);
       led.off();
       delay.delay_ms(10000);
    }
}
