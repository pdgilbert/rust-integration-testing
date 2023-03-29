//! Blink (onboard) LED with short pulse very second and longer blink every ten seconds.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! Two processes are scheduled, `one` for pulse and `ten` for longer blink. These spawn
//! a `blink` process that turns the led on and schedules another process to turn it off.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};

    use systick_monotonic::*;
    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs
    
    use fugit::TimerDuration;

    use cortex_m::asm; //asm::delay(N:u32) blocks the program for at least N CPU cycles.
                       //delay_ms could be used but needs to use a timer other than Systick
                       //use embedded_hal::blocking::delay; //delay::delay_ms(N:u32) blocks the program for N ms.

    const MONOTICK: u32 = 100;
    const ONE: u64 = 1;  // used as seconds
    const TEN: u64 = 10;

    const ONE_DURATION: u64 = 20;  // used as milliseconds
    const TEN_DURATION: u64 = 500;

    use rust_integration_testing_of_examples::led::{setup_led_using_dp, LED, LedType, MONOCLOCK};
    //use rust_integration_testing_of_examples::i2c1_i2c2_led_delay::{
    //    setup_i2c1_i2c2_led_delay_using_dp, I2c2Type, LED, LedType, DelayMs, MONOCLOCK};


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        //rtt_init_print!();
        //rprintln!("blink_rtic example");
        hprintln!("blink_rtic example").unwrap();

        let mut led = setup_led_using_dp(cx.device);
        // Note: the following also woorks but result is a bit larger and bluepill need --release
        //let (_i2c1, _i2c2, mut led, _delay) = setup_i2c1_i2c2_led_delay_using_dp(cx.device);

        led.on();

        // In an application this delay may be while something initializes.
        // Note that this delay cannot use SYST because Monotonics uses that (for spawn,
        //   although spawn has not yet happened so there may be a way?)
        //   delay_ms() would need to use a timer other than default Systick

        asm::delay(5 * MONOCLOCK); // (5 * MONOCLOCK cycles gives aprox 5+ second delay
                               //delay::delay_ms(5_000_u16);

        led.off();

        let mono = Systick::new(cx.core.SYST, MONOCLOCK);

        ten::spawn().unwrap();
        one::spawn().unwrap();
        //hprintln!("init exit").unwrap();

        (Shared { led }, Local {}, init::Monotonics(mono))
    }

    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {}

    #[task(shared = [led], capacity=2)]
    fn one(_cx: one::Context) {
        // blink and re-spawn one process to repeat after ONE second
        blink::spawn(ONE_DURATION.millis()).ok();
        one::spawn_after(ONE.secs()).ok();
    }

    #[task(shared = [led], capacity=2)]
    fn ten(_cx: ten::Context) {
        // blink and re-spawn ten process to repeat after TEN seconds
        // blink on continues after one, otherwise one can turn ten off
        blink::spawn_after(ONE_DURATION.millis(), (TEN_DURATION - ONE_DURATION).millis()).unwrap(); 
        ten::spawn_after(TEN.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        crate::app::led_off::spawn_after(duration).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], capacity=2)]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
