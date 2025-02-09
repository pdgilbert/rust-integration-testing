//  THIS IS NOT USING DelayNs yet

//  CONSIDER USING ALT_DELAY OR
//  CONSIDER SPLITTING INTO SEPARATE VERSIONS FOR EACH SENSOR
//   COMPARE misc-i2c-drivers/htu2xd-display.rs  rtic/htu2xd_rtic.rs
//  AND OTHERS

//  Compare xca9548a which is not rtic

//! Compile with feature hdc1080, or aht10, or aht20, or htu2.
//! eg build
//!   cargo build --no-default-features --target $TARGET --features $MCU,$HAL,aht10 --example temp-humidity-display
//! 
//! or eg run
//!   export INTERFACE=stlink-v2;  openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg 
//! and in another window do
//!   cargo  run --target $TARGET --features $HAL,$MCU,aht10 --example temp-humidity-display  --release
//!
//!
//! Feb 11, 2023 - This compiles and loads (--release) on bluepill and blackpill stm32f401 
//!                    for sensors hdc1080, aht10, and htu2xd.
//!                    with sensor on i2c1 and ssd1306 and ina219 on shared bus i2c2.
//!                    Note: some testing with font ascii::FONT_6X10 and some with iso_8859_1::FONT_9X15
//!              Run testing on bluepill  (scl,sda) i2c1 on (PB8,PB9) and i2c2 on (PB10,PB11).
//!                    with aht10:   works on probe and battery
//!                    with htu2xd:  fails initializing the sensor.
//!                    with hdc1080: works with probe. Unstable and RH reading bad with battery.
//! 
//!              Run testing on blackpill stm32f401 i2c1 on (PB8,PB9) and i2c2 on (PB10,PB3).
//!                    with aht10: works on probe, battery
//!                    with htu2xd:  fails initializing the sensor. (previously have noticed this sensor can
//!                        get in state where power down is needed, but that no longer seems to help.
//!                        Consider software reset of writing registers? Possible htu2xd crate bug.)
//!                    with hdc1080: works with probe and with battery. (and wwith battery charging)
//!
//! 
//! TO DO
//!  -Need error handling for init (and reset for htu2dx?).
//!  -Need better error handling when sensor signal is missed. (ie recover rather than crash)
//!  -test on power through usb plug.
//!  -test on 5v power through ina219. (Configured TP5000 module for the battery chemistry or use usb in place of solar?)
//!  -The ina219 power usage NEEDS CALIBRATION.
//!  -Need to test more with solar power and adjust rtic sleep to check run time possibilities.
//! 
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

//! Compare examples aht10_rtic, ina219-display, hdc1080-display, htu2xd_rtic, xca9548a. 
//! Blink (onboard) LED with short pulse every read.
//! On startup the LED is set on for a second in the init process.
//! One main processe is scheduled. It reads the sensor and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.

//!  Blackpill stm32f401 test wiring see  src/setup_all_stm32f4xx.rs
//! 
//!       This example has a  workaround for SSD1306  text_style.
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;
use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000); 

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [TIM3]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

    // Note that hprintln is for debugging with usb probe and semihosting. 
    // It needs semihosting, which CAUSES BATTERY OPERATION TO STALL.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs


    /////////////////////   ina

    //use  ina219::{INA219,}; //INA219_ADDR
    use ina219::{address::{Address, Pin}, 
                measurements::BusVoltage, 
                SyncIna219, 
                calibration::UnCalibrated,
    };
   

    /////////////////////   TempHumSensor

    // The TempHumSensor trait and methods provide a wrapper so different sensors can be used 
    //  with the same application code.

    pub trait TempHumSensor {
        fn read_th(&mut self) -> (i32, u8);  
          // temp in tenths of deg C, relative humidity
          //(so int rather than float is used for one decimal place in degrees)

        //fn new(i: I2c1Type, d: DelayType) -> Self;   HAVE NOT GOT GENERIC new() TO WORK YET

        fn init(&mut self) -> Result<(),()> {  // for sensors that need it, otherwise default Ok
           Ok(())
        } 

        fn init_message(&mut self, disp: &mut DisplayType) -> ();  
    }

    //////////////////////////////////////////////////////////////////


    #[cfg(feature = "hdc1080")]
    use embedded_hdc1080_rs::{Hdc1080}; 

    #[cfg(feature = "hdc1080")]
    type SensorType = embedded_hdc1080_rs::Hdc1080<I2c1Type, Delay>;
    //type SensorType =  embedded_hdc1080_rs::Hdc1080<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c1Type>>>, DelayType>;

    #[cfg(feature = "hdc1080")]
    type  SensorRcdType = embedded_hdc1080_rs::Hdc1080<RefCellDevice<'static, I2c1Type>, Delay>;

//SENSOR AND INA USE SAME I2C WHICH THUS NEEDS TO BE IN A REFCELL
use core::borrow::BorrowMut;  // bring trait is in scope

    #[cfg(feature = "hdc1080")]
    impl TempHumSensor for SensorRcdType {   // for  SensorType ?
        fn read_th(&mut self) -> (i32, u8) {
           let (t, rh) = match self.borrow_mut().read() {  // .borrow_mut() extracts from RefCell
               Ok((t, rh))     =>((10.0 * t) as i32, rh as u8),
               Err(_e)        => ( -4090, 255)  //supply default values that should be clearly bad
           };
           (t, rh)
        }
        fn init(&mut self) -> Result<(),()> {
           self.borrow_mut().init().unwrap();              // CONSIDER HANDLING OR RETURN ERROR HERE
           Ok(())
        } 

        fn init_message(&mut self, display: & mut DisplayType) -> () {
           show_message("temp-humidity \nhdc1080", display)
        } 
    }



    #[cfg(feature = "htu2")]
    use htu21df_sensor::{Sensor}; 
    //use htu2xd::{Htu2xd, Reading};   //, Resolution

    #[cfg(feature = "htu2")]
    type SensorDev =  htu2xd::Htu2xd<I2c1Type>;
    
    //Workaround. This needs a new struct because channel is not part of the Htu2xd structure

    #[cfg(feature = "htu2")]
    pub struct SensorType { dev: SensorDev, ch: I2c1Type, delay: Delay}


    #[cfg(feature = "htu2")]
    impl TempHumSensor for SensorType {
        fn read_th(&mut self) -> (i32, u8) {
            let t = self.dev.read_temperature_blocking(&mut self.ch);
            let () = t;
 //           let t = match t {
 //               Ok(Reading::Ok(t))     =>  (10.0 * t.as_degrees_celsius()) as i32,
 //               Ok(Reading::ErrorLow)  => -4090,
 //               Ok(Reading::ErrorHigh) => -4090,
 //               Err(_)                 => -4090,
 //           };
            self.delay.delay(15.millis());  // not sure if delay is needed

            let rh = self.dev.read_humidity_blocking(&mut self.ch);
            let () = rh;
  //          let rh = match rh {
  //             Ok(Reading::Ok(rh))    => rh.as_percent_relative() as u8,
  //             Ok(Reading::ErrorLow)  => 255,
  //             Ok(Reading::ErrorHigh) => 255,
  //             Err(_)                 => 255,
  //          };
        match htu.measure_temperature(&mut delay2) {
            Ok(v)     => write!(lines[0], "  {:.1} C? ", v.value()).unwrap(),
            Err(_)    => write!(lines[0], "Error reading temperature").unwrap(),
        }

        match htu.measure_humidity(&mut delay2) {
            Ok(v)     => write!(lines[1], "  {:.1} %RH? ", v.value()).unwrap(),
            Err(_)    => write!(lines[1], "Error reading humidity").unwrap(),
        }

            self.delay.delay(15.millis());  // not sure if delay is needed
        (t, rh)
        }

        // on i2c2 this reset does not return. Temperature reading but not humidity seems to work without it.
        // on i2c1 it gives 'sensor reset failed: ArbitrationLoss'
        //sensor.soft_reset(&mut htu_ch).expect("sensor reset failed");
        //  delay.delay_ms(15u32);     // Wait for the reset to finish
        //    Htu2xd sensor.read_user_register() does not return and changes something that requires sensot power off/on.
        //    let mut register = htu.read_user_register(&mut htu_ch).expect("htu.read_user_register failed");
        //    register.set_resolution(Resolution::Humidity10Temperature13);   //.expect("set_resolution failed");
        //    htu.write_user_register(&mut htu_ch, register).expect("write_user_register failed");

        fn init_message(&mut self, display: & mut DisplayType) -> () {
           show_message("temp-humidity \nhtu2", display)
        } 
    }


    #[cfg(feature = "aht10")]
    use aht10::AHT10;

    #[cfg(feature = "aht10")]
    type SensorType = AHT10<I2c1Type, Delay>;

    #[cfg(feature = "aht10")]
    type  SensorRcdType = AHT10<RefCellDevice<'static, I2c1Type>, Delay>;

    #[cfg(feature = "aht10")]
    impl TempHumSensor for SensorRcdType {
        fn read_th(&mut self) -> (i32, u8) {
            // sensor returns f32 with several decimal places but f32 makes code too large to load on bluepill.
            // 10 * deg C to give one decimal place. (10.0 * t.celsius()) as i32
            let (rh, t) = match self.read() { //return order must be re-arranged
                Ok((rh, t))  =>  (rh.rh() as u8,  (10.0 * t.celsius()) as i32), 
                Err(_e)      =>  {//hprintln!("sensor Error {:?}", e).unwrap(); 
                                  //panic!("Error reading sensor")
                                  (255, -4090)  //supply default values that should be clearly bad
                                 },
            };
            (t, rh)
        }

        fn init_message(&mut self, display: & mut DisplayType) -> () {
           show_message("temp-humidity \nAHT10", display)
        } 
    }


    #[cfg(feature = "aht20")]
    use aht20::Aht20;

    #[cfg(feature = "aht20")]
    type SensorType = Aht20<I2c1Type, Delay>;

    #[cfg(feature = "aht20")]
    type  SensorRcdType = Aht20<RefCellDevice<'static, I2c1Type>, Delay>;

    #[cfg(feature = "aht20")]
    impl TempHumSensor for SensorType {
        fn read_th(&mut self) -> (i32, u8) {
            // sensor returns f32 with several decimal places but f32 makes code too large to load on bluepill.
            // 10 * deg C to give one decimal place. (10.0 * t.celsius()) as i32
            let (rh, t) = match self.read() { //return order must be re-arranged
                Ok((rh, t))  =>  (rh.rh() as u8,  (10.0 * t.celsius()) as i32), 
                Err(_e)      =>  {//hprintln!("sensor Error {:?}", e).unwrap(); 
                                  //panic!("Error reading sensor")
                                  (255, -4090)  //supply default values that should be clearly bad
                                 },
            };
            (t, rh)
        }
        //fn new(i: I2c1Type, d: Delay) -> Self {
        //    Aht20::new(i, d).expect("sensor failed")
        //}

        fn init_message(&mut self, display: & mut DisplayType) -> () {
           show_message("temp-humidity \nAHT20", display)
        } 
    }


    /////////////////////  ssd

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    type  DisplaySize = ssd1306::prelude::DisplaySize128x32;

    //common display sizes are 128x64 and 128x32
    const DISPLAYSIZE: DisplaySize = DisplaySize128x32;

    use embedded_graphics::{
        //mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_9X15 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};



    /////////////////////   constants and types

    const READ_INTERVAL: u32 = 2;  // used as seconds

    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::setup::
          {MONOCLOCK, Peripherals, LED, LedType, I2c1Type, I2c2Type, Delay, DelayNs};

    //    i2c::Error as i2cError,
    //use embedded_hal_async::delay::DelayNs;

    #[cfg(feature = "stm32g4xx")]
    use stm32g4xx_hal::{
        timer::Timer,
        time::{ExtU32, RateExtU32},
        delay::DelayFromCountDownTimer,
    };

    //type  InaType = INA219<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c1Type>>>>;
    //
    //type  DisplayType = ssd1306::Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>, 
    //                         DisplaySize, BufferedGraphicsMode<DisplaySize>>;

    type  InaType = SyncIna219<RefCellDevice<'static, I2c2Type>, UnCalibrated>;

    type  DisplayType = Ssd1306<I2CInterface<RefCellDevice<'static, I2c1Type>>, 
                                 ssd1306::prelude::DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>;


    //use shared_bus::{I2cProxy};
    //use cortex_m::interrupt::Mutex;
    use core::cell::RefCell;
    use embedded_hal_bus::i2c::RefCellDevice;


    //////////////////////////////////////////////////////////////////

    fn show_display<S>(
        temperature: i32,   // deci-degrees = 10 * deg C so integer gives one decimal place
        relative_humidity: u8,
        v: u16,  
        vs: i32,  
        i: i32,  
        p: i32, 
        pc: i32, 
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: ssd1306::size::DisplaySize,  //trait
    {
       let mut line: heapless::String<64> = heapless::String::new();
             
       //write!(lines[0], "{:.1}°C {:.0}% RH", temperature, relative_humidity).unwrap();
       // write!(lines[1], "{:.1}V {}mA {}mW [{}mW]", v as f32/1000.0, i,  p, pc).unwrap();
       //If the ina is wired with Vin- to battery and Vin+ to load sign then the
       // display will show "+" for battery charging and "-" for discharging.
      
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line, "{:3}.{:1}°C {:3}%RH\n{:2}.{:1}V {}mA {}mW [{}mW]", 
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
        S: ssd1306::size::DisplaySize,  //trait
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

    fn read_ina(ina: &mut InaType ) -> (u16, i32, i32, i32)
       {     
        let v = match ina.bus_voltage() {
            Ok(v) => v.voltage_mv(),
            Err(_e)   => 999u16  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let vs = match ina.shunt_voltage() {
            Ok(v) => v.shunt_voltage_uv(),
            Err(_e)   => 999i32  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let i =  0;  //FAKE
        //let i = match ina.current_raw() {
        //    Ok(v) => v.current_ma(),
        //    Err(_e)   => 999i32  //write!(lines[0], "Err: {:?}", e).unwrap()
        //};

        let p = 0;  //FAKE
         //  let p = match ina.power_raw() {  // ina indicated power
         //     Ok(v) => v.power(),
         //     Err(_e)   => 999i32  //write!(lines[0], "Err: {:?}", e).unwrap()
         //  };
                       
        (v, vs, i, p)
       }


    #[shared]
    struct Shared {
        led:   LedType,      //impl LED, would be nice
    }

    #[local]
    struct Local {
        display:  DisplayType,

        ina:  InaType,

        sensor:  SensorRcdType  // error here if feature = "htu2" or "aht10" or other not specified
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        //rtt_init_print!();
        //rprintln!("temp-humidity-display example");
        //hprintln!("temp-humidity-display example").unwrap();

        Mono::start(cx.core.SYST, MONOCLOCK);

        //  sensors use this delay (not systick)
        let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(cx.device);

        led.on();
        //  needs Systick::start first, but also is a future and .await needs to be in a block
        //  Systick::delay(1000.millis());
        delay.delay_ms(1000);
        led.off();

        // As of Feb 2024 I2CDisplayInterface::new is not working with shared bus.
        // This means ssd cannot share i2c bus - unless something other than shared_bus, eg embedded-bus, is used.
        // Best would be for ssd and ina on one i2c bus and the sensor on another
        // because aht10 hardware cannot share bus (without xca9548a or like).

        // Trying embedded-bus
        //    let i2c1_ref_cell = RefCell::new(i2c1);
        //    let sens_rcd = RefCellDevice::new(&i2c1_ref_cell); 
        //    let ina_rcd  = RefCellDevice::new(&i2c1_ref_cell); 
        //    let ssd_rcd  = RefCellDevice::new(&i2c1_ref_cell); 

        //put ssd on i2c1 and shared-bus ina and sensor on i2c2 (will not work for aht10).
        let manager2: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();

        /////////////////////   ssd

        let interface = I2CDisplayInterface::new(i2c1);

        //common display sizes are 128x64 and 128x32
        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        show_message("temp-humidity", &mut display);
        delay.delay_ms(2000);    


        /////////////////////   ina   Start the battery sensor.
       
        //let ina = SyncIna219::new( ina_rcd, Address::from_pins(Pin::Gnd, Pin::Gnd)).unwrap(); 
        let ina = SyncIna219::new(manager2.acquire_i2c(), Address::from_pins(Pin::Gnd, Pin::Gnd)).unwrap(); 

        //let mut ina = INA219::new(ina_rcd, 0x40);
        //hprintln!("let mut ina addr {:?}", INA219_ADDR).unwrap();  // crate's  INA219_ADDR prints as 65

        ina.calibrate(UnCalibrated).unwrap();
        delay.delay_ms(15);     // Wait for sensor

        show_message("battery sensor init", &mut display);   // Example name
        delay.delay_ms(2000);  


        /////////////////////   sens    Start the temp-humidity sensor.

        #[cfg(not(any(feature = "hdc1080", feature = "htu2", feature = "aht10", feature = "aht20")))]
        sensor; // sensor feature must be specified. eg --features $MCU,$HAL,aht10".  COMPILING STOPPED

        #[cfg(feature = "hdc1080")]
        let mut sensor = Hdc1080::new(manager2.acquire_i2c(), delay).unwrap();
        //let mut sensor = Hdc1080::new(sens_rcd, delay).unwrap();

        #[cfg(feature = "htu2")]
        let mut sensor  = SensorType {dev: Htu2xd::new(), ch: sens_rcd, delay};

        #[cfg(feature = "aht10")]
        let mut sensor = AHT10::new(manager2.acquire_i2c(), delay).expect("sensor failed");
        // shared bus would be like this, but does not work for ath10.
        //let mut sensor = AHT10::new(sens_rcd, delay).expect("sensor failed");

        #[cfg(feature = "aht20")]
        let mut sensor = Aht20::new(manager2.acquire_i2c(), delay).expect("sensor failed");
        //let mut sensor = Aht20::new(sens_rcd, delay).expect("sensor failed");

        //let mut sensor = TempHumSensor::<SensorType>::new(sens_rcd, delay).expect("sensor failed");

        sensor.init().unwrap();
        sensor.init_message(&mut display);

        read_and_display::spawn().unwrap();
        //hprintln!("init done").unwrap();

        return(Shared { led, },   Local {display, ina, sensor });
    }

    #[task(shared = [led, ], local = [sensor, ina, display ] )]   //htu_ch, 
    async fn read_and_display(cx: read_and_display::Context) {
       
       let sensor = cx.local.sensor;

       loop {
          Mono.delay_ms(READ_INTERVAL * 1000);  
          //Systick::delay(READ_INTERVAL.secs()).await;
          blink::spawn(BLINK_DURATION).ok();
          //hprintln!("read_and_display").unwrap();
          
          let (t, h) = sensor.read_th();

          let (v, vs, i, p) = read_ina(cx.local.ina);

          // power calculated by P=IV.  (mA x mV /1000 = mW)
          let pc = i as i32 * v as i32 / 1000_i32;     
 
          show_display(t, h,  v, vs, i, p, pc,  cx.local.display);
       }
   }

   #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
        crate::app::led_on::spawn().unwrap();
        Mono.delay_ms(duration); 
        //Systick::delay(duration.millis()).await;
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
