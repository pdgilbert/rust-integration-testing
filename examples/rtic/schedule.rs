// from  /cortex-m-rtic/examples/schedule.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use rtic::app;

// NOTE: does NOT work on QEMU!

// see <pac>::Interrupt for dispatchers,  e.g. stm32l4::stm32l4x1::Interrupt

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))] //fails
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

//#[rtic::app(device = lm3s6965, dispatchers = [SSI0])]

mod app {
    use cortex_m_semihosting::{hprintln};
    use systick_monotonic::*;

    const MONO_HZ: u32 = 8_000_000; // 8 MHz

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        //let systick = cx.core.SYST;

        let mono = Systick::new(cx.core.SYST, MONO_HZ);

        //hprintln!("init").ok();

        //hprintln!("init spawn").ok();
        foo::spawn().ok();
        bar::spawn().ok();
        foo::spawn().ok();
        foo::spawn().unwrap();

        //hprintln!("init spawn_at").ok();
        bar::spawn_at(monotonics::now() + 10.secs()).ok();
        baz::spawn_at(monotonics::now() + 11.secs()).ok();
        foo::spawn_at(monotonics::now() + 12.secs()).unwrap();

        //hprintln!("init spawn_after").ok();
        foo::spawn_after(5.secs()).unwrap();
        baz::spawn_after(6.secs()).ok();
        bar::spawn_after(7.secs()).ok();
        cdr::spawn_after(8.secs()).ok();
        
        hprintln!("init ending").ok();

        (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[task(capacity=5)]
    fn foo(_: foo::Context) {
        hprintln!("foo").ok();
    }

    #[task(capacity=3)]
    fn bar(_: bar::Context) {
        hprintln!("bar").ok();
    }

    #[task(capacity=2)]
    fn baz(_: baz::Context) {
        hprintln!("baz").ok();
    }

    #[task(capacity=1)]
    fn cdr(_: cdr::Context) {
        hprintln!("cdr").ok();
    }
}
