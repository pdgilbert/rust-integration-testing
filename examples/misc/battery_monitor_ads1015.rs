//! Using ads1x1x to read voltages measured by ads1015/1115 ADC on i2c
//! and  crate ssd1306 to print with i2c on a generic ssd1306 based OLED display.
//!
//! Uses the `embedded_graphics` crate to draw.
//! Wiring pin connections for scl and sda to display as in the setup sections below.
//! The full DisplaySize 128x64 is used. Writing will need to be adjusted if display size is smaller.
//! This example took guidance from
//!    https://github.com/jamwaffles/ssd1306/blob/master/examples/text_i2c.rs  and
//!    https://github.com/eldruin/driver-examples/stm32f1-bluepill/examples/ads1015-adc-display-bp.rs
//!  See https://blog.eldruin.com/ads1x1x-analog-to-digital-converter-driver-in-rust/
//!    for much more detailed description.

//! See examples/rtic/battery_monitor_ads1015_rtic.rs  foran rtic version..
//! See examples/misc/battery_monitor_ads1015_rtic_dma.rs for an rtic version using dma bufferring.

// Example use of impl trait: If scl and sda are on PB8 and PB9 (eg in stm32f1xx below) then
//    fn setup() ->  (BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>,
//    PC13<Output<PushPull>>, Delay ) {
// is changed to
//    fn setup() ->  BlockingI2c<I2C1, impl Pins<I2C1>>, PC13<Output<PushPull>>, Delay ) {
// Also
//   use stm32f1xx_hal::{ gpio::{gpiob::{PB8, PB9}, Alternate, OpenDrain, },
// will be needed.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

//use cortex_m_semihosting::{debug, hprintln};
//use cortex_m_semihosting::{hprintln};
//use rtt_target::{rprintln, rtt_init_print};

use ads1x1x::{Ads1x1x, ChannelSelection, DynamicOneShot, FullScaleRange, SlaveAddr};

use core::fmt::Write;

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle, MonoTextStyleBuilder}, //FONT_6X10  FONT_8X13
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use nb::block;

use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};


// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals, },
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (I2cType, LedType, Delay,) {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), &mut rcc);
    let led = setup_led(dp.GPIOC.split(&mut rcc));
    let delay = Delay::new(cp.SYST, &rcc);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (I2cType, LedType, Delay) {
           // (BlockingI2c<I2C2, impl Pins<I2C2>>, impl LED, Delay)
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    //afio  needed for i2c1 (PB8, PB9) but not i2c2
    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &mut afio, &clocks);
    let led = setup_led(dp.GPIOC.split());
    let delay = Delay::new(cp.SYST, &clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals,},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (I2cType, LedType, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);

    let i2c = setup_i2c1(p.I2C1, p.GPIOB.split(&mut rcc.ahb), clocks, rcc.apb1);
    let led = setup_led(p.GPIOE.split(&mut rcc.ahb));
    let delay = Delay::new(cp.SYST, clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    timer::SysDelay as Delay,
    pac::{CorePeripherals, Peripherals,},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (I2cType, LedType, Delay) {
            //(I2c<I2C1, impl Pins<I2C1>>,impl LED, Delay)
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let i2c = setup_i2c1(p.I2C1, p.GPIOB.split(), &clocks);
    //let delay = Delay::new(cp.SYST, &clocks);
    let delay = cp.SYST.delay(&clocks);
    let led = setup_led(p.GPIOC.split());

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals,},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (I2cType, LedType, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);
    let led = setup_led(dp.GPIOC.split());
    let delay = Delay::new(cp.SYST, clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals, },
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (I2cType, LedType, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let i2cx = ccdr.peripheral.I2C1;

    let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
    let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
    let delay = Delay::new(cp.SYST, clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, SCLPin, SDAPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let clocks = rcc.clocks;

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), &mut rcc, &clocks);
    let led = setup_led(dp.GPIOC.split(&mut rcc));
    let delay = Delay::new(cp.SYST, clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    stm32::{CorePeripherals, Peripherals, },
    //gpio::{gpiob::{PB8, PB9}, Output, OpenDrain, },
};

#[cfg(feature = "stm32l1xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (I2cType, LedType, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());
    let clocks = rcc.clocks;

    let gpiob = p.GPIOB.split(&mut rcc);

    // could also have scl,sda  on PB6,PB7 or on PB10,PB11
    // setup_i2c1 NOT WORKING
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    let i2c = p.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);
    // let i2c = setup_i2c1(dp.I2C1, &mut gpiob, rcc);

    let led = setup_led(gpiob.pb6);
    let delay = Delay::new(cp.SYST, clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    pac::{CorePeripherals, Peripherals, },
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
use rust_integration_testing_of_examples::i2c::{setup_i2c1, I2c1Type as I2cType,};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (I2cType, LedType, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
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
    let delay = Delay::new(cp.SYST, clocks);

    (i2c, led, delay) // return tuple (i2c, led, delay)
}

// End of hal/MCU specific setup. Following should be generic code.

pub fn read_all<E, A: DynamicOneShot<Error = E>>(
    adc_a: &mut A,
    adc_b: &mut A,
) -> (i16, i16, i16, [i16; 3]) {
    // Note scale_cur divides, scale_a and scale_b multiplies
    let scale_cur = 10; // calibrated to get mA/mV depends on FullScaleRange above and values of shunt resistors
                        //let scale_a = 2; // calibrated to get mV    depends on FullScaleRange
    let scale_b = 2; // calibrated to get mV    depends on FullScaleRange

    //TMP35 scale is 100 deg C per 1.0v (slope 10mV/deg C) and goes through
    //     <50C, 1.0v>,  so 0.0v is  -50C.

    let scale_temp = 5; //divides
    let offset_temp = 50;

    //first adc  Note that readings are zero on USB power (programming) rather than battery.

    let bat_ma = block!(adc_a.read(ChannelSelection::DifferentialA1A3)).unwrap_or(8091) / scale_cur;
    let load_ma =
        block!(adc_a.read(ChannelSelection::DifferentialA2A3)).unwrap_or(8091) / scale_cur;

    // toggle FullScaleRange to measure battery voltage, not just diff across shunt resistor
    // also first adc
    // Read in main loop and skip here until working see
    //    https://github.com/eldruin/ads1x1x-rs/issues/10?_pjax=%23repo-content-pjax-container
    //adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    //let bat_mv = block!(adc_a.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_a;
    //adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

    // second adc
    let values_b = [
        block!(adc_b.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_b,
        block!(adc_b.read(ChannelSelection::SingleA1)).unwrap_or(8091) * scale_b,
        block!(adc_b.read(ChannelSelection::SingleA2)).unwrap_or(8091) * scale_b,
    ];

    let temp_c =
        block!(adc_b.read(ChannelSelection::SingleA3)).unwrap_or(8091) / scale_temp - offset_temp;

    // third adc
    //let values_c = [
    //    block!(adc_c.read(ChannelSelection::SingleA0)).unwrap_or(8091),
    //    block!(adc_c.read(ChannelSelection::SingleA1)).unwrap_or(8091),
    //    block!(adc_c.read(ChannelSelection::SingleA2)).unwrap_or(8091),
    //    block!(adc_c.read(ChannelSelection::SingleA3)).unwrap_or(8091),
    //];

    //(bat_mv, bat_ma, load_ma, temp_c, values_b )
    (bat_ma, load_ma, temp_c, values_b)
}

fn display<S>(
    bat_mv: i16,
    bat_ma: i16,
    load_ma: i16,
    temp_c: i16,
    values_b: [i16; 3],
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 4] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // it is now possible to use \n in place of separate writes, with one line rather than vector.
    write!(lines[0], "bat:{:4}mV{:4}mA", bat_mv, bat_ma).unwrap();
    write!(lines[1], "load:    {:5}mA", load_ma).unwrap();
    write!(
        lines[2],
        "B:{:4} {:4} {:4}",
        values_b[0], values_b[1], values_b[2]
    )
    .unwrap();
    write!(lines[3], "temperature{:3} C", temp_c).unwrap();

    disp.clear();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 16),
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}


#[entry]
fn main() -> ! {
    let (i2c, mut led, mut delay) = setup();

    led.blink_ok(&mut delay); // blink OK to indicate setup complete and main started.

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    let mut disp = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    disp.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13) //.&FONT_6X10  &FONT_8X13
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        "Display initialized ...",
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut disp)
    .unwrap();

    // For shared_bus::BusManagerSimple the type of next is 
    //   adc_a: Ads1x1x<I2cInterface<I2cProxy<'static, NullMutex<I2c1Type>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
    // That changes a bit in rtic examples because BusManagerSimple cannot be used.

    let mut adc_a = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(false, false)); //addr = GND
    let mut adc_b = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(false, true)); //addr =  V

    // set FullScaleRange to measure expected max voltage.
    // This is very small for diff across low value shunt resistors
    //   but up to 5v for single pin with usb power.
    // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v
    adc_a
        .set_full_scale_range(FullScaleRange::Within0_256V)
        .unwrap();
    //adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    adc_b
        .set_full_scale_range(FullScaleRange::Within4_096V)
        .unwrap();
    //adc_c.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

    let scale_a = 2; // calibrated to get mV    depends on FullScaleRange

    loop {
        // Blink LED to check that everything is actually running.
        // Note that blink takes about a mA current and makes measurement below noisy.
        // Comment out blinking to calibrate scale.
        led.blink(10_u16, &mut delay);

        adc_a
            .set_full_scale_range(FullScaleRange::Within4_096V)
            .unwrap();
        //let bat_mv = block!(adc_a.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_a;
        let bat_mv = block!(DynamicOneShot::read(&mut adc_a, ChannelSelection::SingleA0))
            .unwrap_or(8091)
            * scale_a;
        adc_a
            .set_full_scale_range(FullScaleRange::Within0_256V)
            .unwrap();

        let (bat_ma, load_ma, temp_c, values_b) = read_all(&mut adc_a, &mut adc_b);

        //hprintln!("bat_mv {:4}mV bat_ma {:4}mA  load_ma {:5}mA temp_c {}   values_b {:?}", bat_mv, bat_ma, load_ma, temp_c, values_b).unwrap();

        display(
            bat_mv, bat_ma, load_ma, temp_c, values_b, text_style, &mut disp,
        );

        delay.delay_ms(2000_u16); // sleep for 2s
    }
}
