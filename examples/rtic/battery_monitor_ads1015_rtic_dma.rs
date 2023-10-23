//!  WORK IN PROGRESS FOR ONLY stm32f4xx 
//!    https://github.com/stm32-rs/stm32f4xx-hal/blob/master/examples/rtic-adc-dma.rs
// Following stm32f4xx examples rtic_adc_dma and i2s-audio-out-dma regarding dma use.
// NOT SURE ABOUT THIS. TRYING TO USE DMA FOR EXTERNAL ADC WHEREAS stm32f4xx EXAMPLE IS INTERNAL CHANNEL.
//   MAY  NEED DMA ON I2C INSTEAD.
//! See examples/rtic/battery_monitor_ads1015_rtic.rs  for non-dma version..
//! See examples/misc/battery_monitor_ads1015.rs (not rtic) for details on wiring.
//!  CLEANUP DESCRIPTION. DISPLAY OR LOG???
//! See examples/misc/battery_monitor_ads1015.rs (not rtic) for details on wiring.
//! See examples/misc/battery_monitor_ads1015_rtic_dma.rs for version using dma.

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
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use ads1x1x::{Ads1x1x, DynamicOneShot, FullScaleRange, SlaveAddr, 
                  ChannelSelection,
                  ic::{Ads1015, Resolution12Bit},
                  interface::I2cInterface};
    

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

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
    
    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const MONOTICK: u32 = 100;
    const READ_INTERVAL:  u64 =  2;  // used as seconds
    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    // NEED THIS UNTIL ALL EXAMPLES ARE CONVERTED TO SOMETHING BETTER (as in stm32f4xx)
    #[allow(unused_imports)]

    use rust_integration_testing_of_examples::setups::{
        setup_dht_i2c_led_delay_using_dp, I2cType, LED, LedType, DelayUs, MONOCLOCK};

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        adc::{
            config::{AdcConfig, Dma, SampleTime, Scan, Sequence},
            Adc, //Temperature,
        },
        dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
        pac::{self, ADC1, DMA2},
        prelude::*,
        signature::{VtempCal110, VtempCal30},
    };

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    use nb::block;

    // Note SCALE_CUR divides,  SCALE_B multiplies
    const  SCALE_CUR: i16 = 10; // calibrated to get mA/mV depends on FullScaleRange above and values of shunt resistors
    const  SCALE_A: i16 = 2; // calibrated to get mV    depends on FullScaleRange
    const  SCALE_B: i16 = 2; // calibrated to get mV    depends on FullScaleRange

    //TMP35 scale is 100 deg C per 1.0v (slope 10mV/deg C) and goes through  <50C, 1.0v>,  so 0.0v is  -50C.
    const  SCALE_TEMP:  i16  = 5; //divides
    const  OFFSET_TEMP: i16 = 50;


    fn show_display<S>(
        bat_mv: i16, 
        bat_ma: i16, 
        load_ma: i16, 
        temp_c: i16, 
        values_b: [i16; 3],
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();
    
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
       
       write!(lines[0], "bat {:3}mv bat {:3}ma load {:3}ma temp{:3}°C v1{:3} v2{:3}  v3{:3} FIX ME", 
                         bat_mv, bat_ma, load_ma, temp_c, values_b[0], values_b[1], values_b[2],).unwrap();

       disp.clear_buffer();
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

    type DMATransfer =
        Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 2]>;


    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, MONOCLOCK);

        //rtt_init_print!();
        //rprintln!("battery_monitor_ads1015_rtic example");
        hprintln!("battery_monitor_ads1015_rtic example").unwrap();

        let (_dht, i2c, mut led, mut delay) = setup_dht_i2c_led_delay_using_dp(cx.device);

        led.on(); 
        delay.delay_ms(1000u32);
        led.off();

        let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

        let interface = I2CDisplayInterface::new(manager.acquire_i2c());

        let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();

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
            Err(e) =>  {hprintln!("Error {:?} in adc_b.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

////
     //   let gpiob = cx.device.GPIOB.split();
     //   let voltage = gpiob.pb1.into_analog();

        let dma = StreamsTuple::new(cx.device.DMA2);

        let config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(false);

        let adc_config = AdcConfig::default()
            .dma(Dma::Continuous)
            .scan(Scan::Enabled);

        let mut adc = Adc::adc1(cx.device.ADC1, true, adc_config);
        adc.configure_channel(&adc_a, Sequence::One, SampleTime::Cycles_480);
        adc.configure_channel(&adc_b, Sequence::Two, SampleTime::Cycles_480);
        //adc.enable_temperature_and_vref();

        let first_buffer = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
        let second_buffer = Some(cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap());
        let transfer = Transfer::init_peripheral_to_memory(dma.0, adc, first_buffer, None, config);

     //   polling::spawn_after(1.secs()).ok();
////
        read_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();

        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();

        (
            Shared {led, transfer },
            Local {
                adc_a, adc_b, display,
                buffer: second_buffer,
            },
            init::Monotonics(mono),
        )
    }


    #[shared]
    struct Shared {
        led: LedType,
        transfer: DMATransfer,
    }

    #[local]
    struct Local {
       adc_a:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       adc_b:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       display: Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, 
                          ssd1306::prelude::DisplaySize128x64, 
                          BufferedGraphicsMode<DisplaySize128x64>>,
//        text_style: MonoTextStyle<BinaryColor>,
       buffer: Option<&'static mut [u16; 2]>,
    }

    #[idle(local = [])]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). It may affect debugging.
           rtic::export::wfi()
        }
    }

    #[task(shared = [led], local = [adc_a, adc_b, display], capacity=4)]
    fn read_and_display(mut cx: read_and_display::Context) {
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

        show_display(bat_mv, bat_ma, load_ma, temp_c, values_b, &mut cx.local.display);
        
        hprintln!("bat_mv {:4}mV bat_ma {:4}mA  load_ma {:5}mA temp_c {}   values_b {:?}", bat_mv, bat_ma, load_ma, temp_c, values_b).unwrap();
 
        read_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();
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
