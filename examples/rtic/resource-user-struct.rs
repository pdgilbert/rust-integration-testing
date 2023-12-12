//  from  /cortex-m-rtic/examples/resource.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac  ))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac  ))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac  ))] //fails:  USART1 variant or associated item not found in `stm32f3xx_hal::interrupt
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac  ))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac  ))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac  ))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32  ))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac  ))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac  ))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac  ))]

//#[rtic::app(device = lm3s6965)]

mod app {
    use cortex_m_semihosting::{debug, hprintln};

    //use lm3s6965::Interrupt;

    #[cfg(feature = "stm32f0xx")]
    use stm32f0xx_hal::pac::Interrupt as interrupt;

    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::pac::Interrupt as interrupt;

    //#[cfg(feature = "stm32f3xx")]
    //use stm32f3xx_hal::interrupt;

    #[cfg(feature = "stm32f3xx")]
    mod interrupt {
        pub use stm32f3xx_hal::interrupt::*;
        pub use stm32f3xx_hal::interrupt::{USART1_EXTI25 as USART1, USART2_EXTI26 as USART2};
    }

    //#[cfg(feature = "stm32f3xx")]
    //use stm32f3xx_hal::interrupt::{USART1_EXTI25  as USART1,
    //                               USART2_EXTI26  as USART2};

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::interrupt;

    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::interrupt;

    #[cfg(feature = "stm32g0xx")]
    use stm32g0xx_hal::interrupt;

    #[cfg(feature = "stm32g4xx")]
    use stm32g4xx_hal::interrupt;

    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::interrupt;

    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::pac::Interrupt as interrupt;

    #[cfg(feature = "stm32l1xx")]
    use stm32l1xx_hal::interrupt;

    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::interrupt;

    #[shared]
    struct Shared {
        // A resource
        shared: u32,
    }

    // Should not collide with the struct above
    #[allow(dead_code)]
    struct Shared2 {
        // A resource
        shared: u32,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local ) {
        rtic::pend(interrupt::USART1);
        rtic::pend(interrupt::USART2);

        (Shared { shared: 0 }, Local {}, init::Monotonics())
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("about to  debug::EXIT_SUCCESS").unwrap();
        debug::exit(debug::EXIT_SUCCESS);

        // error: no `shared` field in `idle::Context`
        // _cx.shared.shared += 1;

        loop {}
    }

    // `shared` can be accessed from this context
    #[task(binds = USART1, shared = [shared])]
    fn usart1(mut cx: usart1::Context) {
        let shared = cx.shared.shared.lock(|shared| {
            *shared += 1;
            *shared
        });

        hprintln!("USART1: shared = {}", shared).unwrap();
    }

    // `shared` can be accessed from this context
    #[task(binds = USART2, shared = [shared])]
    fn usart2(mut cx: usart2::Context) {
        let shared = cx.shared.shared.lock(|shared| {
            *shared += 1;
            *shared
        });

        hprintln!("USART2: shared = {}", shared).unwrap();
    }
}
