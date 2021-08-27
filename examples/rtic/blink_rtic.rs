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

use rtic::app;


// NOT SURE IF  peripherals = true, IS USED ?

#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   peripherals = true, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   peripherals = true, dispatchers = [TIM2, TIM3]))] 
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   peripherals = true, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   peripherals = true, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   peripherals = true, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, peripherals = true, dispatchers = [TIM2, TIM3]))] 
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   peripherals = true, dispatchers = [TIM2, TIM3]))]


mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::{Seconds, Milliseconds};


    const ONE: Seconds = Seconds(1); 
    const TEN: Seconds = Seconds(10);
    
    const ONE_DURATION: Milliseconds = Milliseconds(20); 
    const TEN_DURATION: Milliseconds = Milliseconds(500);


    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},  //, State},
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f1xx")]
    type LedType = PC13<Output<PushPull>>;
    
    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut gpioc = dp.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self)  -> () { self.set_low() }
            fn off(&mut self) -> () { self.set_high()}
        }
    
        led
    }


    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
       gpio::{gpioe::PE15, Output, PushPull},
        pac::Peripherals,
        prelude::*,
    };
    
    #[cfg(feature = "stm32f3xx")]
    type LedType = PE15<Output<PushPull>>;
    
    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc = dp.RCC.constrain();
        //let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);
    
        let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
        let mut led = gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    
        impl LED for PE15<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_high().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_low().unwrap()
            }
        }
        led.off();
    
        led
    }
    
    
    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
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
                self.set_low()
            }
            fn off(&mut self) -> () {
                self.set_high()
            }
        }
    
        led
    }

    
    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
    };
    
    #[cfg(feature = "stm32f7xx")]
    type LedType = PC13<Output<PushPull>>;
    
    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) -> LedType {
        //let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();
    
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
    
    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
    };
    
    #[cfg(feature = "stm32h7xx")]
    use embedded_hal::digital::v2::OutputPin;
    
    #[cfg(feature = "stm32h7xx")]
    type LedType = PC13<Output<PushPull>>;
    
    #[cfg(feature = "stm32h7xx")]
    fn setup(dp: Peripherals) -> LedType {
        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate

    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
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

    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
        rcc, // for ::Config but note name conflict with serial
    };
    
    #[cfg(feature = "stm32l0xx")]
    type LedType = PC13<Output<PushPull>>;
    
    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
        let gpioc = p.GPIOC.split(&mut rcc);
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
    
    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        gpio::{gpiob::PB6, Output, PushPull},
        prelude::*,
        stm32::Peripherals,
    };
    
    #[cfg(feature = "stm32l1xx")]
    use embedded_hal::digital::v2::OutputPin;
    
    #[cfg(feature = "stm32l1xx")]
    type LedType = PB6<Output<PushPull>>;
    
    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) -> LedType {
        let gpiob = dp.GPIOB.split();
        let led = gpiob.pb6.into_push_pull_output();
    
        impl LED for PB6<Output<PushPull>> {
            fn on(&mut self) -> () {
                self.set_high().unwrap()
            }
            fn off(&mut self) -> () {
                self.set_low().unwrap()
            }
        }
    
        led
    }
        
    
    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
    };
    
    #[cfg(feature = "stm32l4xx")]
    use embedded_hal::digital::v2::OutputPin;
    
    #[cfg(feature = "stm32l4xx")]
    type LedType = PC13<Output<PushPull>>;
    
    #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc   = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    
        impl LED for PC13<Output<PushPull>> {
            fn on(&mut self)  -> () { self.set_low().unwrap() }
            fn off(&mut self) -> () { self.set_high().unwrap()}
        }
    
        led
    }
    
    // End of hal/MCU specific setup. Following should be generic code.  
    
    
    pub trait LED {
        // depending on board wiring, on may be set_high or set_low, with off also reversed
        // implementation should deal with this difference
        fn on(&mut self) -> ();
        fn off(&mut self) -> ();
    }
    
    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = DwtSystick<80_000_000>;
    
    //#[init(schedule = [one, ten])]

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, 8_000_000);

        //rtt_init_print!();
        //rprintln!("blink example");

        let mut led = setup(cx.device);
    
        led.off();

        ten::spawn().unwrap();

        // may need small time offset to avoid collision
        one::spawn().unwrap();
    
       (Shared {led}, Local {}, init::Monotonics(mono))
    }
    
    #[shared]
    struct Shared {
        led: LedType,
    }
    
    #[local]
    struct Local {}
    
    //#[task(resources = [led], spawn = [blink], schedule = [one] )]

    #[task(shared = [led])]
    fn one(_cx: one::Context) {
        // blink and re-spawn one process to repeat after ONE second
        blink::spawn(ONE_DURATION).ok();
        one::spawn_after(ONE).ok();
    }

    //#[task(resources = [led], spawn = [blink], schedule = [ten] )]
    #[task(shared = [led])]
    fn ten(_cx: ten::Context) {
        // blink and re-spawn ten process to repeat after TEN seconds
        blink::spawn(TEN_DURATION).ok();
        ten::spawn_after(TEN).ok();
    }
     
    //#[task(priority = 2, capacity = 3 , spawn = [led_on], schedule = [led_off] )]

    #[task(shared = [led])]
    fn blink(_cx: blink::Context, duration: Milliseconds<u32>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        crate::app::led_off::spawn_after(duration).ok();
        crate::app::led_on::spawn().ok();
    }
   
    //#[task(priority = 3, capacity = 3, resources = [led] )]

    #[task(shared = [led])]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }
    
    //#[task(priority = 3, capacity = 3, resources = [led] )]

    #[task(shared = [led])]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
