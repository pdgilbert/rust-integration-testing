//! Jan, 2023 - This is working with USB probe and with battery. Run tested on blackpill.
//!       Note that battery current will only be non-zero when running on battery.
//!       It is zero when running on the probe.             
//! 
//!       This example has a  workaround for SSD1306  text_style.
//! 
//! Measure battery current and display on OLED with shared bus on i2c2. With minor changes
//! this also works on i2c1. Some sensor only work
//! on i2c1 so other example need this to be on i2c2 and it is used here of consistency.
//! This example monitors the current to/from the battery. Using something like a TP5000 module,
//! configured for battery chemistry, a solar panel or usb charger can be attached to the
//! 

//! The SSD1306 display and the ina219 battery monitoring are the same 12cbus.
//! (Temp/humidity sensor htu21d's  humidity reading seems to conflict with ina219 (error reading 
//! humidity) and AHT10 does not work with anything else on the same i2c bus so having display and
//! ina219 together on one bus is for future considerations.)

//! Compare examples ina219-display, htu2xd_rtic, temp-humidity-display. 
//! See data sheet at https://www.ti.com/product/INA219
//! Blink (onboard) LED with short pulse evry read.
//! On startup the LED is set on for a second in the init process.
//! One main processe is scheduled. It reads the sensor and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.

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
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use  ina219::{INA219,}; //INA219_ADDR

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
    
type  DisplayType = ssd1306::prelude::DisplaySize128x32;

//common display sizes are 128x64 and 128x32
const DISPLAYSIZE: DisplayType = DisplaySize128x32;

const VPIX:i32 = 16;  // vertical pixels for a line, including space

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        //mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const READ_INTERVAL: u32 = 2;  // used as seconds

    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::i2c1_i2c2_led;
    use rust_integration_testing_of_examples::i2c1_i2c2_led::{ I2c2Type, LED, LedType, DelayNs, MONOCLOCK};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    fn show_display<S>(v: u16,  vs: i16,  i: i16,  p: i16, 
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
  
       // power caclulated by P=IV.  (mA x mV /1000 = mW)
       //If the ina is wired with Vin- to battery and Vin+ to load sign then the
       // display will show "+" for battery charging and "-" for discharging.
       let pc = i as i32 * v as i32 / 1000_i32;

       write!(lines[0], "V: {}mv Vs: {}mV", v, vs).unwrap();
       write!(lines[1], "I: {}mA P:{}mW [{}mW]", i,  p, pc).unwrap();

       disp.clear_buffer();
       for i in 0..lines.len() {
           // start from 0 requires that the top is used for font baseline
           Text::with_baseline(
               &lines[i],
               Point::new(0, i as i32 * VPIX), 
               text_style,
               Baseline::Top,
               )
               .draw(&mut *disp)
               .unwrap();
       }
       disp.flush().unwrap();
       ()
    }


    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        //rtt_init_print!();
        //rprintln!("htu2xd_rtic example");
        //hprintln!("htu2xd_rtic example").unwrap();

        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        let (_i2c1, i2c2, mut led, _clock) = i2c1_i2c2_led::setup(cx.device);

        led.on();
        Systick.delay_ms(1000u32);  
        led.off();

        let manager2: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();
        let interface = I2CDisplayInterface::new(manager2.acquire_i2c());

        let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

        //common display sizes are 128x64 and 128x32
        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        Text::with_baseline(   "INA219-rtic", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
        display.flush().unwrap();
        
        Systick.delay_ms(2000u32);    

        // Start the battery sensor.
        let mut ina = INA219::new(manager2.acquire_i2c(), 0x40);
        //hprintln!("let mut ina addr {:?}", INA219_ADDR).unwrap();  // crate's  INA219_ADDR prints as 65
        ina.calibrate(0x0100).unwrap(); // possible change. See the data sheet.
        Systick.delay_ms(15u32);     // Wait for sensor

        read_and_display::spawn().unwrap();

        (Shared { led, },   Local {display, ina })
    }

    #[shared]
    struct Shared {
        led:   LedType,      //impl LED, would be nice
    }

    #[local]
    struct Local {
        display:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>, 
                             DisplayType, BufferedGraphicsMode<DisplayType>>,
        ina:  INA219<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c2Type>>>>,
    }

    #[task(shared = [led, ], local = [ina, display ] )]
    async fn read_and_display(cx: read_and_display::Context) {
       
       let ina = cx.local.ina;

       loop { 
           blink::spawn(BLINK_DURATION).ok();

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
           

           show_display(v, vs, i, p, cx.local.display);
           
           Systick::delay(READ_INTERVAL.secs()).await;
       }
    }

    #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
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
