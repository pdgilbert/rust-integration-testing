//! Compile with feature hdc1080, or aht10, or htu2xd.


//! Feb 4, 2023 - This compiles and works on blackpill stm32f401  with sensor hdc1080 on i2c1
//!                and with ina219 and sdd1306 on shared bus i2c2. Tested with probe and with battery.
//!                The power usage  RESULTS NEED CALIBRATION.
//! 
//!  Feb 8, 2023 - AHT10 is working. Also on bluepill
//!
//!       Note that battery current will only be non-zero when running on battery.
//!       It is zero when running on the probe.             
//! 
//! This example reads temperature and humidity from a sensor and display on an OLED SSD1306 display.
//! The current to/from the battery is also measured and displayed on SSD1306 with shared bus i2c2.
//! A TP5000 module or similar can receive power from a solar panel or usb charger. Its output
//! attaches to the MCU and other modules, and to the battery through the ina219 measurement.
//! The TP5000 module should be configured for the battery chemistry.
//! 
//! The SSD1306 display and the ina219 (https://www.ti.com/product/INA219) battery monitoring are the
//! same 12cbus. (Temp/humidity sensor htu21d's  humidity reading seems to conflict with ina219 (error 
//! reading humidity) and AHT10 does not work with anything else on the same i2c bus so having display
//! and ina219 together on one bus is for future considerations.)

//! Compare examples aht10_rtic, ina219-display, hdc1080-display, htu2xd_rtic. 
//! Blink (onboard) LED with short pulse every read.
//! On startup the LED is set on for a second in the init process.
//! One main processe is scheduled. It reads the sensor and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.

//!  Blackpill stm32f401 test wiring:
//!     SSD1306 and ina219 on   shared bus  i2c2   sda on B3   scl on B10
//!             sensor     on   shared bus  i2c1   sda on B8   scl on B9 
//! 
//!       This example has a  workaround for SSD1306  text_style.
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]

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
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use  ina219::{INA219,}; //INA219_ADDR


    #[cfg(feature = "hdc1080")]
    use embedded_hdc1080_rs::{Hdc1080}; 

    #[cfg(feature = "hdc1080")]
    type SensorType = embedded_hdc1080_rs::Hdc1080<I2c1Type, DelayType>;
    //type SensorType =  embedded_hdc1080_rs::Hdc1080<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c1Type>>>, DelayType>;

 
    #[cfg(feature = "htu2xd")]
    use htu2xd::{Htu2xd, Reading};   //, Resolution

    #[cfg(feature = "htu2xd")]
    type SensorType =  htu2xd::Htu2xd<I2c1Type>;


    #[cfg(feature = "aht10")]
    use aht10::AHT10;

    #[cfg(feature = "aht10")]
    type SensorType = AHT10<I2c1Type, DelayType>;


    // Note that hprintln is for debugging with usb probe and semihosting. 
    // It needs semihosting, which CAUSES BATTERY OPERATION TO STALL.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;

    use systick_monotonic::*;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs
    use fugit::TimerDuration;


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

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        //mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const MONOTICK:  u32 = 100;
    const READ_INTERVAL: u64 = 2;  // used as seconds

    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    // DelayType is only needed for some sensors. It gives a warningfor others.
    use rust_integration_testing_of_examples::i2c1_i2c2_led_delay::{
        setup_i2c1_i2c2_led_delay_using_dp, I2c1Type, I2c2Type, LED, LedType, DelayMs, DelayType, MONOCLOCK};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    fn show_display<S>(
        temperature: i32,   // deci-degrees = 10 * deg C so integer gives one decimal place
        relative_humidity: u8,
        v: u16,  _vs: i16,  i: i16,  p: i16, 
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       let mut line: heapless::String<64> = heapless::String::new();
             
       // power calculated by P=IV.  (mA x mV /1000 = mW)
       //If the ina is wired with Vin- to battery and Vin+ to load sign then the
       // display will show "+" for battery charging and "-" for discharging.
       let pc = i as i32 * v as i32 / 1000_i32;
       
       //write!(lines[0], "{:.1}°C {:.0}% RH", temperature, relative_humidity).unwrap();
      // write!(lines[1], "{:.1}V {}mA {}mW [{}mW]", v as f32/1000.0, i,  p, pc).unwrap();
      
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line, "{:3}.{:1}°C {:3}% RH\n{:2}.{:1}V {}mA {}mW [{}mW]", 
            temperature/10, temperature%10, relative_humidity, v/1000, (10*(v%1000))/1000, i,  p, pc).unwrap();
            //temperature/10, temperature%10, relative_humidity, v as f32/1000.0, i,  p, pc).unwrap();

       show_message(&line, disp);
        ()
    }

    fn show_message<S>(
        text: &str,   //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       disp.clear();
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


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        //rtt_init_print!();
        //rprintln!("temp-humidity-display example");
        //hprintln!("temp-humidity-display example").unwrap();

        //let mut led = setup(cx.device);
        let (i2c1, i2c2, mut led, mut delay) = setup_i2c1_i2c2_led_delay_using_dp(cx.device);

        led.on();
        delay.delay_ms(1000u32);  
        led.off();

        let manager2: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();
        let interface = I2CDisplayInterface::new(manager2.acquire_i2c());

        //let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

        //common display sizes are 128x64 and 128x32
        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        #[cfg(not(any(feature = "hdc1080", feature = "htu2xd", feature = "aht10")))]
        sensor; // sensor must be specified. crash

        #[cfg(feature = "hdc1080")]
        show_message("temp-humidity\nHDC1080", &mut display);

        #[cfg(feature = "htu2xd")]
        show_message("temp-humidity \nHTU2XD", &mut display);

        #[cfg(feature = "aht10")]
        show_message("temp-humidity \nAHT10", &mut display);


        delay.delay_ms(2000u32);    

        // Start the battery sensor.
        let mut ina = INA219::new(manager2.acquire_i2c(), 0x40);
        //hprintln!("let mut ina addr {:?}", INA219_ADDR).unwrap();  // crate's  INA219_ADDR prints as 65
        ina.calibrate(0x0100).unwrap();
        delay.delay_ms(15u32);     // Wait for sensor
        show_message("battery sensor init", &mut display);   // Example name
        delay.delay_ms(2000u32);  


        // Start the temp-humidity sensor.

        //let manager1: &'static _ = shared_bus::new_cortexm!(I2c1Type = i2c1).unwrap();

        #[cfg(feature = "hdc1080")]
        let mut sensor = Hdc1080::new(i2c1, delay).unwrap();
        //let mut sensor = Hdc1080::new(manager1.acquire_i2c(), delay).unwrap();

        #[cfg(feature = "hdc1080")]
        sensor.init().unwrap();

        #[cfg(feature = "htu2xd")]
        let sensor    = Htu2xd::new();

        #[cfg(feature = "htu2xd")]
        let htu_ch = i2c1;
//        let mut htu_ch = manager1.acquire_i2c();
//        // on i2c2 this reset does not return. Temperature reading but not humidity seems to work without it.
//        // on i2c1 it gives 'sensor reset failed: ArbitrationLoss'
//        #[cfg(feature = "htu2xd")]
//        //sensor.soft_reset(&mut htu_ch).expect("sensor reset failed");
//        delay.delay_ms(15u32);     // Wait for the reset to finish
        //    Htu2xd sensor.read_user_register() does not return and changes something that requires sensot power off/on.
        //    let mut register = htu.read_user_register(&mut htu_ch).expect("htu.read_user_register failed");
        //    register.set_resolution(Resolution::Humidity10Temperature13);   //.expect("set_resolution failed");
        //    htu.write_user_register(&mut htu_ch, register).expect("write_user_register failed");


        #[cfg(feature = "aht10")]
        let sensor = AHT10::new(i2c1, delay).expect("sensor failed");


        let mono = Systick::new(cx.core.SYST,  MONOCLOCK);

        read_and_display::spawn().unwrap();
        //hprintln!("init done").unwrap();

        #[cfg(feature = "hdc1080")]
        return(Shared { led, },   Local {display, ina, sensor }, init::Monotonics(mono));

        #[cfg(feature = "htu2xd")]
        return(Shared { led, },   Local {display, ina, sensor, htu_ch }, init::Monotonics(mono));

        #[cfg(feature = "aht10")]
        return(Shared { led, },   Local {display, ina, sensor }, init::Monotonics(mono));
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

        sensor:  SensorType,

        #[cfg(feature = "htu2xd")]
        htu_ch:  I2c1Type,
        //htu_ch:  I2cProxy<'static, Mutex<RefCell<I2c1Type>>>,
    }

    #[task(shared = [led, ], local = [sensor, htu_ch, ina, display ], capacity=2)]   //htu_ch, 
    fn read_and_display(cx: read_and_display::Context) {
        blink::spawn(BLINK_DURATION.millis()).ok();
        //hprintln!("read_and_display").unwrap();
        
        //let delay = cx.local.delay;
        let sensor = cx.local.sensor;

        #[cfg(feature = "hdc1080")]
        let (t, h) = match sensor.read() {
            Ok((t, h))     =>(t, h),
            Err(_e)        => {show_message("sensor read error ", cx.local.display);
                               (409.1, 409.2)
                              }
        };
 

        #[cfg(feature = "htu2xd")]
        let z = sensor.read_temperature_blocking(cx.local.htu_ch);    // VERY SLOW RETURNING 409

        #[cfg(feature = "htu2xd")]
        let t = match z {
            Ok(Reading::Ok(t))     => t.as_degrees_celsius(),
            Ok(Reading::ErrorLow)  => 409.0,
            Ok(Reading::ErrorHigh) => 409.1,
            Err(_)                 => 409.2,
        };
 
        #[cfg(feature = "htu2xd")]
        let z = sensor.read_humidity_blocking(cx.local.htu_ch);

        #[cfg(feature = "htu2xd")]
        let h = match z {
            Ok(Reading::Ok(t))     => t.as_percent_relative(),
            Ok(Reading::ErrorLow)  => 409.0,
            Ok(Reading::ErrorHigh) => 409.1,
            Err(_)                 => 409.2,
        };

        // sensor returns f32 with several decimal places but f32 makes code too large to load on bluepill.
        // 10 * deg C to give one decimal place. (10.0 * t.celsius()) as i32
        #[cfg(feature = "aht10")]
        let (h, t) = match sensor.read() {
            Ok((h, t))
               =>  (h.rh() as u8,  (10.0 * t.celsius()) as i32), //integer in deci-degrees to show 1 decimal place
            Err(_e) 
               =>  {//hprintln!("sensor Error {:?}", e).unwrap(); 
                    //panic!("Error reading sensor")
                    (255, -4090)  //supply default values that should be clearly bad
                   },
        };

        let (v, vs, i, p) = read_ina(cx.local.ina);

        show_display(t, h,  v, vs, i, p,  cx.local.display);

        read_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
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
