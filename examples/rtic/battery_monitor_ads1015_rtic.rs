//!  CLEANUP DESCRIPTION. DISPLAY OR LOG???
// CONSIDER DOING THIS WITH DMA FOLLOWING stm32f4xx examples adc_dma_rtic and i2s-audio-out-dma.rs
//! See examples/misc/battery_monitor_ads1015.rs (not rtic) for details on wiring.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_rtt_target as _;

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
    //use rtt_target::{rprintln, rtt_init_print};

    use ads1x1x::{Ads1x1x, DynamicOneShot, FullScaleRange, SlaveAddr, 
                  ChannelSelection,
                  ic::{Ads1015, Resolution12Bit},
                  interface::I2cInterface};
    //use core::fmt::Write;

    use systick_monotonic::*;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs

    use fugit::TimerDuration;

    const MONOTICK: u32 = 100;
    const READ_INTERVAL:  u64 =  2;  // used as seconds
    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};

    // A delay is used in some sensor initializations and read. 
    // Systick is used by monotonic (for spawn), so delay needs to use a timer other than Systick
    // asm::delay used in AltDelay is not an accurate timer but gives a delay at least 
    //  number of indicated clock cycles.
    use rust_integration_testing_of_examples::alt_delay::{AltDelay};

    // set up for shared bus
   // use shared_bus_rtic::SharedBus;
    //use shared_bus_rtic::export::interrupt::Mutex;
    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;
    //  or      rtic::Mutex;
    //  or      rtic::export::interrupt::Mutex;

    use embedded_graphics::{
        mono_font::{ascii::FONT_8X13, MonoTextStyle, MonoTextStyleBuilder}, //FONT_6X10  FONT_8X13
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    use nb::block;



    #[cfg(feature = "stm32f0xx")]
    use stm32f0xx_hal::{
        pac::Peripherals,
        prelude::*,
    };
 
    #[cfg(feature = "stm32f0xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f0xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c2, I2c2Type as I2cType,};

    #[cfg(feature = "stm32f0xx")]
    fn setup(mut dp: Peripherals) ->  (I2cType, LedType, AltDelay) {    
       let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

       let i2c = setup_i2c2(dp.I2C2, dp.GPIOB.split(&mut rcc),  &mut rcc);

       let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
       led.off();

       let delay = AltDelay{};

       (i2c, led, delay)
    }
    


    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        pac::Peripherals, //I2C1
        prelude::*,
    };

    #[cfg(feature = "stm32f1xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();
        let clocks = rcc
            .cfgr
            //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
            //.sysclk(72.mhz()) // system clock 8 MHz default, max 72MHz
            //.pclk1(36.mhz())  // system clock 8 MHz default, max 36MHz ?
            .freeze(&mut flash.acr);
        //let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

        hprintln!("hclk {:?}",   clocks.hclk()).unwrap();
        hprintln!("sysclk {:?}", clocks.sysclk()).unwrap();
        hprintln!("pclk1 {:?}",  clocks.pclk1()).unwrap();
        hprintln!("pclk2 {:?}",  clocks.pclk2()).unwrap();
        hprintln!("pclk1_tim {:?}", clocks.pclk1_tim()).unwrap();
        hprintln!("pclk2_tim {:?}", clocks.pclk2_tim()).unwrap();
        hprintln!("adcclk {:?}",    clocks.adcclk()).unwrap();
        //hprintln!("usbclk_valid {:?}", clocks.usbclk_valid()).unwrap(); not fo all MCUs

        let gpiob = dp.GPIOB.split();

        //afio  needed for i2c1 (PB8, PB9) but not i2c2
        let i2c = setup_i2c1(dp.I2C1, gpiob, &mut afio, &clocks);

        let mut led = setup_led(dp.GPIOC.split()); 
        led.off();

        let delay = AltDelay{};

        (i2c, led, delay)
    }

    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        pac::{Peripherals, },
        prelude::*,
    };

    #[cfg(feature = "stm32f3xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> (I2cType, LedType, AltDelay) {
       let mut flash = dp.FLASH.constrain();
       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze(&mut flash.acr);
    
       let gpiob = dp.GPIOB.split(&mut rcc.ahb);
       let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);

       let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
       led.off();

       let delay = AltDelay{};

       (i2c, led, delay)
    }

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        pac::{Peripherals, },
        prelude::*,
    };

    #[cfg(feature = "stm32f4xx")]
    const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f4xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
       let rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       let delay = AltDelay{};

       (i2c, led, delay)
    }


    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        pac::{Peripherals, },
        prelude::*,
    };

    #[cfg(feature = "stm32f7xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();
       //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

       let led = setup_led(dp.GPIOC.split());
       let delay = AltDelay{};

      (i2c, led, delay)
    }

    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32h7xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32h7xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32h7xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
       let pwr = dp.PWR.constrain();
       let vos = pwr.freeze();
       let rcc = dp.RCC.constrain();
       let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
       let clocks = ccdr.clocks;

       let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
       let i2cx = ccdr.peripheral.I2C1;  //.I2C4;

       let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
       let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
       let delay = AltDelay{};

       (i2c, led, delay)
    }

    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull, OpenDrain, gpioa::PA8},
        pac::Peripherals,
        prelude::*,
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l0xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l0xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l0xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
       // UNTESTED
       let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
       let clocks = rcc.clocks;

       let mut dht = dp.GPIOA.split(&mut rcc).pa8.into_open_drain_output();
 
       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), dp.AFIO.constrain(), &clocks);
       let led = setup_led(dp.GPIOC.split(&mut rcc));
       let delay = AltDelay{};

       (i2c, led, delay)
    }


    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        prelude::*,
        rcc::Config as rccConfig,
        stm32::{Peripherals},
    };

    #[cfg(feature = "stm32l1xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
       let mut rcc = dp.RCC.freeze(rccConfig::hsi());

       let gpiob = dp.GPIOB.split(&mut rcc);

// setup_i2c1 NOT WORKING
       let scl = gpiob.pb8.into_open_drain_output();
       let sda = gpiob.pb9.into_open_drain_output(); 
       let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);
//       let i2c = setup_i2c1(dp.I2C1, gpiob, rcc);

       let led = setup_led(gpiob.pb6);
       let delay = AltDelay{};

       (i2c, led, delay)
    }

    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        pac::{Peripherals,},
        prelude::*,
    };

    #[cfg(feature = "stm32l4xx")]
    const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l4xx")]
    use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

    #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) ->  (I2cType, LedType, AltDelay) {
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
        let clocks = rcc
            .cfgr
            .sysclk(80.mhz())
            .pclk1(80.mhz())
            .pclk2(80.mhz())
            .freeze(&mut flash.acr, &mut pwr);

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
       let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
       let delay = AltDelay{};

       (i2c, led, delay)
    }

    // End of hal/MCU specific setup. Following should be generic code.


    // Note SCALE_CUR divides,  SCALE_B multiplies
    const  SCALE_CUR: i16 = 10; // calibrated to get mA/mV depends on FullScaleRange above and values of shunt resistors
    const  SCALE_A: i16 = 2; // calibrated to get mV    depends on FullScaleRange
    const  SCALE_B: i16 = 2; // calibrated to get mV    depends on FullScaleRange

    //TMP35 scale is 100 deg C per 1.0v (slope 10mV/deg C) and goes through  <50C, 1.0v>,  so 0.0v is  -50C.
    const  SCALE_TEMP:  i16  = 5; //divides
    const  OFFSET_TEMP: i16 = 50;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, MONOCLOCK);

        //rtt_init_print!();
        //rprintln!("battery_monitor_ads1015_rtic example");
        hprintln!("battery_monitor_ads1015_rtic example").unwrap();

        let (i2c, mut led, mut delay) = setup(cx.device);
   
        led.on(); 
        delay.delay_ms(1000u32);
        led.off();

        let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

        let interface = I2CDisplayInterface::new(manager.acquire_i2c());

        let text_style = MonoTextStyleBuilder::new().font(&FONT_8X13).text_color(BinaryColor::On).build();

        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        delay.delay_ms(2000u32);

        display.init().unwrap();

        Text::with_baseline("Display initialized ...", Point::zero(), text_style, Baseline::Top, )
           .draw(&mut display).unwrap();

        let mut adc_a = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(false, false)); //addr = GND
        let mut adc_b = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(false, true)); //addr =  V

        // set FullScaleRange to measure expected max voltage.
        // This is very small for diff across low value shunt resistors for current
        //   but up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        match adc_a.set_full_scale_range(FullScaleRange::Within0_256V) {
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_a.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

        match adc_b.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_2.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

        measure::spawn_after(READ_INTERVAL.secs()).unwrap();

        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();

        (Shared {led}, 
         Local {adc_a, adc_b, }, 
         init::Monotonics(mono)
        )
    }

    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {
       adc_a: Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
        adc_b: Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
    }

    #[idle(local = [])]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). It may affect debugging.
           rtic::export::wfi()
        }
    }

    #[task(shared = [led], local = [adc_a, adc_b,], capacity=4)]
    fn measure(cx: measure::Context) {
       //hprintln!("measure").unwrap();
       blink::spawn(BLINK_DURATION.millis()).ok();

       cx.local.adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();  // reading voltage which is higher 
       let bat_mv = block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::SingleA0)).unwrap_or(8091)* SCALE_A;
       cx.local.adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

       //first adc  Note that readings will be zero using USB power (ie while programming) 
       // but not when using battery.

       let bat_ma =
           block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::DifferentialA2A3)).unwrap_or(8091) / SCALE_CUR;

       let load_ma =
           block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::DifferentialA2A3)).unwrap_or(8091) / SCALE_CUR;

       // second adc
       let values_b = [
           block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA0)).unwrap_or(8091) * SCALE_B,
           block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA1)).unwrap_or(8091) * SCALE_B,
           block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA2)).unwrap_or(8091) * SCALE_B,
       ];

       let temp_c =
           block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA3)).unwrap_or(8091) / SCALE_TEMP - OFFSET_TEMP;

        //showDisplay(bat_mv, bat_ma, load_ma, temp_c, values_b, text_style, &mut disp);
        
        hprintln!("bat_mv {:4}mV bat_ma {:4}mA  load_ma {:5}mA temp_c {}   values_b {:?}", bat_mv, bat_ma, load_ma, temp_c, values_b).unwrap();
 
        measure::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
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
