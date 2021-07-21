//! examples/schedule.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m::peripheral::DWT;
use cortex_m_semihosting::hprintln;
use panic_semihosting as _;
use rtic::cyccnt::{Instant, U32Ext as _};

// NOTE: does NOT work on QEMU!
//#[rtic::app(device = lm3s6965, monotonic = rtic::cyccnt::CYCCNT)]
#[rtic::app(device = stm32f1xx_hal::pac,  monotonic = rtic::cyccnt::CYCCNT)]
//#[rtic::app(device = stm32f4xx_hal::pac,  monotonic = rtic::cyccnt::CYCCNT)]

const APP: () = {
    #[init(schedule = [foo, bar])]
    fn init(mut cx: init::Context) {
        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        DWT::unlock();
        cx.core.DWT.enable_cycle_counter();

        // semantically, the monotonic timer is frozen at time "zero" during `init`
        // NOTE do *not* call `Instant::now` in this context; it will return a nonsense value
        let now = cx.start; // the start time of the system

        hprintln!("init @ {:?}", now).unwrap();

        // Schedule `foo` to run 8e6 cycles (clock cycles) in the future
        cx.schedule.foo(now + 8_000_000.cycles()).unwrap();

        // Schedule `bar` to run 4e6 cycles in the future
        cx.schedule.bar(now + 4_000_000.cycles()).unwrap();
    }

    #[task]
    fn foo(_: foo::Context) {
        hprintln!("foo  @ {:?}", Instant::now()).unwrap();
    }

    #[task]
    fn bar(_: bar::Context) {
        hprintln!("bar  @ {:?}", Instant::now()).unwrap();
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.

    //#[cfg(feature = "stm32f1xx")]   CFG DOES NOT WORK HERE
    extern "C" {
        fn TIM2();
        fn TIM3();
        fn TIM4();
        fn QEI0();
    }

    //    #[cfg(feature = "stm32f4xx")]CFG DOES NOT WORK HERE
    //    extern "C" {
    //        fn EXTI0();
    //        fn SDIO();
    //        fn QEI0();
    //    }
};
