//! NOT YET WORKING. NEED RTIC VERSION OF SSD1306 FOR OLED. COMPARE display_stuff_rtic
//! Measure the temperature and humidity from a DHT11 or DHT22 on data pin (A8) and display on OLED with i2c.
//! (Specify feature "dht22"for DHT22).
//! Compare examples oled_dht, and blink_rtic.
//! Blink (onboard) LED with short pulse very read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main processe is scheduled. It reads the dht and spawns itself to run after aa delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;

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

    //https://github.com/michaelbeaumont/dht-sensor
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::Reading;
    #[cfg(feature = "dht22")]
    use dht_sensor::dht22::Reading;
    use dht_sensor::*;
        
    use cortex_m::asm; //asm::delay(N:u32) blocks the program for at least N CPU cycles.
                       //delay_ms could be used but needs to use a timer other than Systick
                       //use embedded_hal::blocking::delay; //delay::delay_ms(N:u32) blocks the program for N ms.

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};
    
    use fugit::TimerDuration;

    const MONOTICK: u32 = 100;
    const READ_INTERVAL: u64 = 10;  // used as seconds

    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull, //, State},
               gpioa::PA8, OpenDrain},
        device::I2C2,
        i2c::{BlockingI2c, DutyCycle, Mode, Pins},
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    type LedType = PC13<Output<PushPull>>;

    #[cfg(feature = "stm32f1xx")]
    fn setup(dp: Peripherals) ->  (PA8<Output<OpenDrain>>, BlockingI2c<I2C2, impl Pins<I2C2>>, LedType, Delay) {
          //let mut gpioc = dp.GPIOC.split();
          //let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
       // This delay used for dht initialization and read cannot be systick which is used by spawn.
       let mut delay = asm::delay(5 * CLOCK); // (5 * CLOCK cycles gives aprox 5+ second delay

       let mut gpioa = dp.GPIOA.split();

       let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
       dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
       delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

       let mut gpiob = dp.GPIOB.split();

       let rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);
       let i2c = BlockingI2c::i2c2(
           dp.I2C2,
           (
               gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
               gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
           ),
           //&mut afio.mapr,  need this for i2c1 but not i2c2
           Mode::Fast {
               frequency: 400_000.hz(),
               duty_cycle: DutyCycle::Ratio2to1,
           },
           clocks,
           1000,
           10,
           1000,
           1000,
       );

       let led = setup_led(dp.GPIOC.split());
    
       (dht, i2c, led, delay)
       }

    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        gpio::{gpioe::PE15, Output, PushPull},
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32f3xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    type LedType = PE15<Output<PushPull>>;

    #[cfg(feature = "stm32f3xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc = dp.RCC.constrain();
        //let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

        let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
        let mut led = gpioe
            .pe15
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

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
    const CLOCK: u32 = 16_000_000; //should be set for board not for HAL

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
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    type LedType = PC13<Output<PushPull>>;

    #[cfg(feature = "stm32f7xx")]
    fn setup(dp: Peripherals) -> LedType {
        //let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

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

    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        pac::Peripherals,
        prelude::*,
    };

    #[cfg(feature = "stm32h7xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32h7xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

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
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

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
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l1xx")]
    use embedded_hal::digital::v2::OutputPin;

    #[cfg(feature = "stm32l1xx")]
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    type LedType = PB6<Output<PushPull>>;

    #[cfg(feature = "stm32l1xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
        let gpiob = dp.GPIOB.split(&mut rcc);
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
    const CLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l4xx")]
    type LedType = PC13<Output<PushPull>>;

    #[cfg(feature = "stm32l4xx")]
    fn setup(dp: Peripherals) -> LedType {
        let mut rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
        let led = gpioc
            .pc13
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

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

    // End of hal/MCU specific setup. Following should be generic code.


// 
//    pub trait LED {
//        // depending on board wiring, on may be set_high or set_low, with off also reversed
//        // implementation should deal with this difference
//        fn on(&mut self) -> ();
//        fn off(&mut self) -> ();
//    }
use rust_integration_testing_of_examples::i2c_led_delay::{setup_led, LED};


fn show_display<S>(
    temperature: i8,
    relative_humidity: u8,
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 1] = [
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // It is possible to use \n in place of separate writes, with one line rather than vector.

    // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
    // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
    // the line commented out!! The test_txt is taken from 
    //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
    
    //let test_text  = "¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
    //   degree symbol "°" is about                  ^^ here 
    
    write!(lines[0], "{:3}°C {:3}% RH", temperature, relative_humidity).unwrap();
 
    disp.clear();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        //rtt_init_print!();
        //rprintln!("blink_rtic example");
        hprintln!("blink_rtic example").unwrap();

        //let mut led = setup(cx.device);
        let (mut dht, i2c, mut led, mut delay) = setup(cx.device);

            led.on();

        let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);
        let interface = I2CDisplayInterface::new(manager.acquire());

        //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        led.off();

        let mono = Systick::new(cx.core.SYST, CLOCK);

        read_and_display::spawn().unwrap();

        (Shared { led, delay, dht, text_style, display }, Local {}, init::Monotonics(mono))
    }

    #[shared]
    struct Shared {
        led: LedType,
        delay: Delay,
        dht: PA8<Output<OpenDrain>>,
        text_style: MonoTextStyle<BinaryColor>,
        display: &mut Ssd1306<impl WriteOnlyDataCommand, DisplaySize, BufferedGraphicsMode<DisplaySize>>,
    }

    #[local]
    struct Local {}

    #[task(shared = [led, delay, dht, text_style, display], capacity=2)]
    fn read_and_display(cx: read_and_display::Context) {
        // blink and re-spawn process to repeat
        blink::spawn(BLINK_DURATION.millis()).ok();

        //match Reading::read(&mut delay, &mut dht) {
        //let z = Reading::read(&mut delay, &mut dht); 
        let z = cx.shared.dht.lock(|dht| Reading::read(&mut delay, &mut dht));
        match z {
            Ok(Reading {
                temperature,
                relative_humidity,}) 
               => {//hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                   show_display(temperature, relative_humidity, text_style, &mut display)},
            Err(_e) 
               =>  {//hprintln!("Error {:?}", e).unwrap(); 
                    panic!("Error reading DHT")},
        }

        read_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led, delay], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        crate::app::led_off::spawn_after(duration).unwrap();
    }

    #[task(shared = [led, delay], capacity=2)]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led, delay], capacity=2)]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
