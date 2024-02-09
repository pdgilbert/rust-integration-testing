// Originally from  /cortex-m-rtic/examples/schedule.rs. This is reworked for rtic v2. 

// NEED TO ADD MORE CHECKS FOR SCHEDULING

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_semihosting as _;

use rtic::app;

// NOTE: does NOT work on QEMU!

// see <pac>::Interrupt for dispatchers,  e.g. stm32l4::stm32l4x1::Interrupt

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

//#[rtic::app(device = lm3s6965, dispatchers = [SSI0])]

mod app {
    use cortex_m_semihosting::{hprintln};

    use rtic;
    use rtic_monotonics::systick::{Systick};
    use rtic_monotonics::systick::fugit::{ExtU32};

    const MONOCLOCK: u32 = 8_000_000; // 8 MHz

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {

        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        //hprintln!("init").ok();

        //hprintln!("init spawn").ok();
        foobar1::spawn().unwrap();
        foobar2::spawn().unwrap();
        foobar3::spawn().unwrap();

        //hprintln!("init spawn_at").ok();
        //bar::spawn_at(monotonics::now() + 10.secs()).ok();
        //baz::spawn_at(monotonics::now() + 11.secs()).ok();
        //foo::spawn_at(monotonics::now() + 12.secs()).unwrap();

        //hprintln!("init spawn_after").ok();
        //foo::spawn_after(5.secs()).unwrap();
        //baz::spawn_after(6.secs()).ok();
        //bar::spawn_after(7.secs()).ok();
        //cdr::spawn_after(8.secs()).ok();
        
        hprintln!("init ending").ok();

        (Shared {}, Local {})
    }

    #[task( )]
    async fn foobar1(_: foobar1::Context) {
        foo::spawn().ok();
        bar::spawn().ok();
        foo::spawn().ok();
        foo::spawn().unwrap();
    }

    // TRY TO DO SOMETHING MORE LIKE SPAWN_AT AND SPAWN_AFTER
    #[task( )]
    async fn foobar2(_: foobar2::Context) {
       Systick::delay(0.secs() + 10.secs()).await;  //0 in place of now()
       foo::spawn().unwrap();

       Systick::delay(1.secs()).await;
       baz::spawn().ok();

       Systick::delay(1.secs()).await;
       bar::spawn().ok();

       Systick::delay(1.secs()).await;
       cdr::spawn().ok();
    }

    #[task( )]
    async fn foobar3(_: foobar3::Context) {
       Systick::delay(1.secs()).await;
       baz::spawn().ok();

       Systick::delay(1.secs()).await;
       bar::spawn().ok();

       Systick::delay(1.secs()).await;
       cdr::spawn().ok();
    }


    #[task( )]
    async fn foo(_: foo::Context) {
       hprintln!("foo").ok();
    }

    #[task( )]
    async fn bar(_: bar::Context) {
       hprintln!("bar").ok();
    }

    #[task( )]
    async fn baz(_: baz::Context) {
       Systick::delay(1000.millis()).await;
       hprintln!("baz").ok();
    }

    #[task( )]
    async fn cdr(_: cdr::Context) {
       Systick::delay(1000.millis()).await;
       hprintln!("cdr").ok();
   }
}
