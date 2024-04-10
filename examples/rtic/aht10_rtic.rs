     THIS REQUIRES SHARING THE BUS ina canshare but aht10 hardware wants it's own bus and
     ssd is not working with shared_bus, so cannot work without ssd fixed (or 3 i2c's)
     
//!   NOT HARDWARE TESTED SINCE EMBEDDED-HAL V1.0.0 CHANGES
//!
//! Feb 6, 2023 - This is working with USB probe and with battery. 
//!              Run tested on bluepill  (scl,sda) i2c1 on (PB8,PB9) and i2c2 on (PB10,PB11).
//!                    with aht10 on i2c1 and ssd1306 on i2c2.
//!                    with aht10 on i2c1 and ssd1306 on shared bus i2c2.
//!                    with aht10 on i2c1 and ssd1306 and ina on shared bus i2c2.
//! 
//!              Run tested on blackpill stm32f401 i2c1 on (PB8,PB9) and i2c2 on (PB10,PB3).
//!                    with aht10 on i2c1 and ssd1306 on i2c2.
//!                    with aht10 on i2c1 and ssd1306 on shared bus i2c2.
//!                    with aht10 on i2c1 and ssd1306 and ina on shared bus i2c2.
//! 
//!             It has a  workaround for SSD1306  text_style.
//! 
//! Note that led and i2c pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.
//! 
//! Measure the temperature and humidity from an AHT10 on i2c2.
//! Measure the battery with ina219 on i2c1 and display on OLED with ssd1306 on i2c1.
//!     
//! Compare examples dht_rtic, aht10_displau.
//! Blink (onboard) LED with short pulse evry read.
//! On startup the LED is set on for a second in the init process.
//! One main processe is scheduled. It reads the sensor and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [TIM3]))]
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

mod app {
    use aht10::AHT10;  //or aht10-async
    use ina219::{INA219,}; //INA219_ADDR

    // Note that hprintln is for debugging with usb probe and semihosting. It causes battery operation to stall.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;

    use rtic;
    use rtic_monotonics::systick::Systick;
    use rtic_monotonics::systick::fugit::{ExtU32};

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const READ_INTERVAL: u32 = 2;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::setup::{MONOCLOCK, I2c1Type, I2c2Type, LED, LedType, Delay};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    fn show_display<S>(
        temperature: i32,   // 10 * deg C to give one decimal place
        relative_humidity: u8,
        v: u16,  _vs: i16,  i: i16,  p: i16, 
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {    
       let mut line: heapless::String<64> = heapless::String::new();
    
       // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
       // It is possible to use \n in place of separate writes, with one line rather than vector.
    
       // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
       // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
       // the line commented out!! The test_txt is taken from 
       //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
       
       //let test_text  = "¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
       //   degree symbol "°" is about                  ^^ here 

       let pc = i as i32 * v as i32 / 1000_i32;
       
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line, "{:3}.{:1}°C {:3}% RH\n{:2}.{:1}V {}mA {}mW [{}mW]", 
            temperature/10, temperature%10, relative_humidity, v/1000, (10*(v%1000))/1000, i,  p, pc).unwrap();
            //temperature/10, temperature%10, relative_humidity, v as f32/1000.0, i,  p, pc).unwrap();

       show_message(&line, disp);
      ()
    }

    fn show_message<S>(
       text: &str, 
       disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {         
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       disp.clear_buffer();
       Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
               .draw(&mut *disp)
               .unwrap();

       disp.flush().unwrap();
       ()
    }


    fn read_ina(ina: &mut INA219<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c2Type>>>>
          ) -> (u16, i16, i16, i16)
       {

        let v = match ina.voltage() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let vs = match ina.shunt_voltage() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let i = match ina.current() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let p = match ina.power() {  // ina indicated power
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };
                 
        (v, vs, i, p)
       }


    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {

        let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(cx.device);

        led.on();
        delay.delay(1000.millis());  
        led.off();

        let manager: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();
        let interface = I2CDisplayInterface::new(manager.acquire_i2c());
        //let interface = I2CDisplayInterface::new(i2c2);  //without shared bus

        //common display sizes are 128x64 and 128x32
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        show_message("AHT10_rtic", &mut display);   // Example name
        
        delay.delay(2000.millis());  

        // Start the battery sensor.
        let mut ina = INA219::new(manager.acquire_i2c(), 0x40);
        //hprintln!("let mut ina addr {:?}", INA219_ADDR).unwrap();  // crate's  INA219_ADDR prints as 65
        ina.calibrate(0x0100).unwrap();

        delay.delay(15.millis());     // Wait for sensor
        show_message("battery sensor init", &mut display);   // Example name
        delay.delay(2000.millis());  

        // aht10 hardware does not allow sharing the bus, so second bus is used.
        //  See example aht10_display for more details
        let sensor = AHT10::new(i2c1, delay).expect("sensor failed"); //NEEDS NON SYSTICK DELAY
        show_message("sensor initialized", &mut display); 

        read_and_display::spawn().unwrap();

        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        (Shared { led, },   Local {display, ina,  sensor })
    }

    #[shared]
    struct Shared { led:   LedType, }

// see disply_stuff_rtic and  https://github.com/jamwaffles/ssd1306/issues/164 regarding
// problem Shared local text_style. Workaroound by building in the task (may not be very efficient).

    #[local]
    struct Local {
        display:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>, 
//        display:  Ssd1306<I2CInterface<I2c2Type>, 
                          ssd1306::prelude::DisplaySize128x32, 
                          BufferedGraphicsMode<DisplaySize128x32>>,

        ina:  INA219<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c2Type>>>>,

        sensor:  AHT10<I2c1Type, Delay>,
    }

    #[task(shared = [led, ], local = [sensor, display, ina] )]
    async fn read_and_display(cx: read_and_display::Context) {
       
       //hprintln!("read_and_display").unwrap();
       let sensor = cx.local.sensor;
       
       loop {
           blink::spawn(BLINK_DURATION).ok();

            let z = sensor.read();
           // sensor returns f32 with several decimal places but f32 makes code too large to load on bluepill.
           // 10 * deg C to give one decimal place.
           let (h, t) = match z {
               Ok((h, t))
                  =>  (h.rh() as u8,  (10.0 * t.celsius()) as i32),
               Err(_e) 
                  =>  {//hprintln!("sensor Error {:?}", e).unwrap(); 
                       //panic!("Error reading sensor")
                       (127, 127)  //supply default values that should be clearly bad
                      },
           };

           let (v, vs, i, p) = read_ina(cx.local.ina);

           show_display(t, h, v, vs, i, p, cx.local.display);
           //hprintln!("shown").unwrap();

           Systick::delay(READ_INTERVAL.secs()).await;
       }
    }

    #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        Systick::delay(duration.millis()).await;
        crate::app::led_off::spawn().unwrap();
    }

    #[task(shared = [led] )]
    async fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led] )]
    async fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
