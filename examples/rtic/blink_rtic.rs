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
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
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

    use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};



    #[cfg(feature = "stm32f0xx")]
    use stm32f0xx_hal::{
        pac::Peripherals,
        prelude::*,
    };
 
    #[cfg(feature = "stm32f0xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f0xx")]
    fn setup(mut dp: Peripherals) -> LedType {    
       let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

       let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
       led.off();

       led
    }


    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) -> LedType {
       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       led
    }

    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f3xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> LedType {
       let mut rcc = dp.RCC.constrain();

       let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
       led.off();

       led
    }

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f4xx")]
    const CLOCK: u32 = 16_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    fn setup(dp: Peripherals) -> LedType {
       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       led
    }

    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f7xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) -> LedType {
       let led = setup_led(dp.GPIOC.split());
       
       led
    }

    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
       pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32h7xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32h7xx")]
    fn setup(dp: Peripherals) -> LedType {
       let pwr = dp.PWR.constrain();
       let vos = pwr.freeze();
       let rcc = dp.RCC.constrain();
       let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
       let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));

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
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) -> LedType {
      let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
      let led = setup_led(dp.GPIOC.split(&mut rcc));

      led
    }

    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        prelude::*,
        stm32::Peripherals,
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) -> LedType {
       let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
       let gpiob = dp.GPIOB.split(&mut rcc);
       let led = setup_led(gpiob.pb6);

       led
    }

    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32l4xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

   #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) -> LedType {
       let mut rcc = dp.RCC.constrain();
       let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));

       led
    }

    // End of hal/MCU specific setup. Following should be generic code.



    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        //rtt_init_print!();
        //rprintln!("blink_rtic example");
        hprintln!("blink_rtic example").unwrap();

        let mut led = setup(cx.device);

        led.on();

        // In an application this delay may be while something initializes.
        // Note that this delay cannot use SYST because Monotonics uses that (for spawn,
        //   although spawn has not yet happened so there may be a way?)
        //   delay_ms() would need to use a timer other than default Systick

        asm::delay(5 * CLOCK); // (5 * CLOCK cycles gives aprox 5+ second delay
                               //delay::delay_ms(5_000_u16);

        led.off();

        let mono = Systick::new(cx.core.SYST, CLOCK);

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
