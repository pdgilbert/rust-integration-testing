// from  /cortex-m-rtic/examples/schedule.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use rtic::app;

// NOTE: does NOT work on QEMU!

// see <pac>::Interrupt for dispatchers,  e.g. stm32l4::stm32l4x1::Interrupt

#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]  //fails
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

//#[rtic::app(device = lm3s6965, dispatchers = [SSI0])]

mod app {
    use cortex_m_semihosting::hprintln;
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::Seconds;

    const MONO_HZ: u32 = 8_000_000; // 8 MHz

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<MONO_HZ>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dcb = cx.core.DCB;
        let dwt = cx.core.DWT;
        let systick = cx.core.SYST;

        let mono = DwtSystick::new(&mut dcb, dwt, systick, 8_000_000);

        hprintln!("init").ok();

        // Schedule `foo` to run 1 second in the future
        foo::spawn_after(Seconds(1_u32)).ok();

        // Schedule `bar` to run 2 seconds in the future
        bar::spawn_after(Seconds(2_u32)).ok();

        (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[task]
    fn foo(_: foo::Context) {
        hprintln!("foo").ok();
    }

    #[task]
    fn bar(_: bar::Context) {
        hprintln!("bar").ok();
    }
}
