//! Blink (onboard) LED with short pulse very second and longer blink every ten seconds.
//! Two processes are scheduled, `one` for pulse and `ten` for longer blink. These spawn
//! a `blink` process that turns the led on and schedules another process to turn it off.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use embedded_hal::digital::v2::OutputPin;

use rtic::app;
use rtic::cyccnt::U32Ext;

pub trait LED {
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // implementation should deal with this difference
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();
}

// NOTE CYCCNT IS 32 bit at cpu freq and NOT GOOD FOR ORDER OF MAGNITUDE OF SECONDS ?? RTIC 1.4 Timer Queue

#[cfg(feature = "stm32f1xx")]
const MILLISECOND: u32 = 8_000; // Duration of a millisecond in cyccnt (syst ticks)

#[cfg(feature = "stm32f4xx")]
const MILLISECOND: u32 = 16_000; // Duration of a millisecond in cyccnt (syst ticks)

const PULSE: u32 = 1_000 * MILLISECOND; // 1 second
const PERIOD: u32 = 10 * PULSE; // 10 seconds

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull, State},
    //pac,
    pac::Peripherals,
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f1xx")]
fn setup(dp: Peripherals) -> LedType {
    let mut rcc = dp.RCC.constrain();

    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc
        .pc13
        .into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
    //let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }
    led.off();

    led
}

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    pac,
    pac::Peripherals,
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f4xx")]
fn setup(dp: Peripherals) -> LedType {
    let gpioc = dp.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    led
}

//  NEED TO SPECIFY DEVICE HERE FOR DIFFERENT HALs

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
//#[app(device = stm32f4xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]

const APP: () = {
    struct Resources {
        led: LedType,
    }

    #[init(schedule = [one, ten])]
    fn init(cx: init::Context) -> init::LateResources {
        //rtt_init_print!();
        //rprintln!("blink example");

        let mut core = cx.core;
        core.DCB.enable_trace();
        cortex_m::peripheral::DWT::unlock(); // needed on some devices
        core.DWT.enable_cycle_counter();

        let device: Peripherals = cx.device;

        let mut led = setup(device);

        led.off();

        cx.schedule.ten(cx.start + PERIOD.cycles()).unwrap();
        // offset 700 ms so blinks do not overlap
        cx.schedule
            .one(cx.start + (PULSE - 700 * MILLISECOND).cycles())
            .unwrap();

        init::LateResources { led }
    }

    #[task(resources = [led], spawn = [blink], schedule = [one] )]
    fn one(cx: one::Context) {
        cx.spawn.blink(5u32).unwrap();
        cx.schedule.one(cx.scheduled + PULSE.cycles()).unwrap();
    }

    #[task(resources = [led], spawn = [blink], schedule = [ten] )]
    fn ten(cx: ten::Context) {
        cx.spawn.blink(500u32).unwrap();
        cx.schedule.ten(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    #[task(priority = 3, capacity = 3, resources = [led] )]
    fn led_on(cx: led_on::Context) {
        cx.resources.led.on();
    }

    #[task(priority = 3, capacity = 3, resources = [led] )]
    fn led_off(cx: led_off::Context) {
        cx.resources.led.off();
    }

    #[task(priority = 2, capacity = 3 , spawn = [led_on], schedule = [led_off] )]
    fn blink(cx: blink::Context, time: u32) {
        //time in ms
        cx.schedule
            .led_off(cx.scheduled + (time * MILLISECOND).cycles())
            .unwrap();
        cx.spawn.led_on().unwrap();
    }

    // HOW TO KNOW USED INTERRUPTS?
    //#[cfg(feature = "stm32f1xx")]   CFG DOES NOT WORK HERE
    extern "C" {
        //        fn EXTI0();
        //        fn SDIO();
        fn TIM2();
        fn TIM3();
        fn TIM4();
        fn QEI0();
    }

    //    #[cfg(feature = "stm32f4xx")]CFG DOES NOT WORK HERE
    //    extern "C" {
    //        fn EXTI0();
    //        fn SDIO();
    //        fn TIM1();
    //        fn TIM2();
    //        fn TIM3();
    //        fn QEI0();
    //    }
};
