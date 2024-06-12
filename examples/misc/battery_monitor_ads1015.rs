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

//! See examples/misc-i2c-drivers/ina219-display.rs another (better) method to monitor power usage.
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

//use embedded_hal::delay::DelayNs;

/////////////////////   ads
use ads1x1x::{Ads1x1x, channel, FullScaleRange, SlaveAddr, mode::OneShot,
              ic::{Ads1015, Resolution12Bit}};

/////////////////////   ssd
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle, MonoTextStyleBuilder}, //FONT_6X10  FONT_8X13
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};


///////////////////// 
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;


/////////////////////  setup
use rust_integration_testing_of_examples::setup;  // setup function
use rust_integration_testing_of_examples::setup::{Peripherals, LED, block, DelayNs, I2c1Type };


/////////////////////  
//Ads1x1x<embedded_hal_bus::i2c::RefCellDevice<'_, rust_integration_testing_of_examples::setup::I2c<I2C1, PB9<AlternateOD<4>>, PB8<AlternateOD<4>>>>, Ads1015, Resolution12Bit, OneShot>
//Ads1x1x<RefCellDevice<'_, I2c<I2C1, PB9<AlternateOD<4>>, PB8<AlternateOD<4>>>>, Ads1015, Resolution12Bit, OneShot>
//pub fn read_all<E, A: DynamicOneShot<Error = E>>(
pub fn read_all(
    adc_a: &mut Ads1x1x<RefCellDevice<'_, I2c1Type>, Ads1015, Resolution12Bit, OneShot>,
    adc_b: &mut Ads1x1x<RefCellDevice<'_, I2c1Type>, Ads1015, Resolution12Bit, OneShot>,
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

    let bat_ma = block!(adc_a.read(channel::DifferentialA1A3)).unwrap_or(8091) / scale_cur;
    let load_ma =
        block!(adc_a.read(channel::DifferentialA2A3)).unwrap_or(8091) / scale_cur;

    // toggle FullScaleRange to measure battery voltage, not just diff across shunt resistor
    // also first adc
    // Read in main loop and skip here until working see
    //    https://github.com/eldruin/ads1x1x-rs/issues/10?_pjax=%23repo-content-pjax-container
    //adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    //let bat_mv = block!(adc_a.read(ChannelSelection::SingleA0)).unwrap_or(8091) * scale_a;
    //adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

    // second adc
    let values_b = [
        block!(adc_b.read(channel::SingleA0)).unwrap_or(8091) * scale_b,
        block!(adc_b.read(channel::SingleA1)).unwrap_or(8091) * scale_b,
        block!(adc_b.read(channel::SingleA2)).unwrap_or(8091) * scale_b,
    ];

    let temp_c =
        block!(adc_b.read(channel::SingleA3)).unwrap_or(8091) / scale_temp - offset_temp;

    // third adc
    //let values_c = [
    //    block!(adc_c.read(channel::SingleA0)).unwrap_or(8091),
    //    block!(adc_c.read(channel::SingleA1)).unwrap_or(8091),
    //    block!(adc_c.read(channel::SingleA2)).unwrap_or(8091),
    //    block!(adc_c.read(channel::SingleA3)).unwrap_or(8091),
    //];

    //(bat_mv, bat_ma, load_ma, temp_c, values_b )
    (bat_ma, load_ma, temp_c, values_b)
}

fn show_display<S>(
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


#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    
    let (i2cset, mut led, mut delay) = setup::i2c_led_delay_from_dp(dp);

    led.blink_ok(&mut delay); // blink OK to indicate setup complete and main started.

    let i2cset_ref_cell = RefCell::new(i2cset);
    let adc_a_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let adc_b_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

    /////////////////////   ads

    //let mut adc_a = Ads1x1x::new_ads1015(adc_a_rcd, SlaveAddr::default()); //addr = GND
    let mut adc_a = Ads1x1x::new_ads1015(adc_a_rcd, SlaveAddr::Gnd);
    let mut adc_b = Ads1x1x::new_ads1015(adc_b_rcd, SlaveAddr::Vdd);
    //let mut adc_c = Ads1x1x::new_ads1015(adc_c_rcd, SlaveAddr::Sda);
    //let mut adc_d = Ads1x1x::new_ads1015(adc_d_rcd, SlaveAddr::Scl);

    // set FullScaleRange to measure expected max voltage.
    // This is very small for diff across low value shunt resistors
    //   but up to 5v for single pin with usb power.
    // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

    adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();
    //adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

    adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    //adc_c.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
    //let interface = I2CDisplayInterface::new_custom_address(ssd_rcd,   0x3D);  //alt address

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

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
    .draw(&mut display)
    .unwrap();

    /////////////////////

    let scale_a = 2; // calibrated to get mV    depends on FullScaleRange

    loop {
        // Blink LED to check that everything is actually running.
        // Note that blink takes about a mA current and makes measurement below noisy.
        // Comment out blinking to calibrate scale.
        led.blink(10_u16, &mut delay);

        adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

        let bat_mv = block!(adc_a.read(channel::SingleA0)).unwrap_or(8091) * scale_a;
        //let bat_mv = block!(DynamicOneShot::read(&mut adc_a, ChannelSelection::SingleA0))
        //    .unwrap_or(8091)
        //    * scale_a;

        adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();
        let (bat_ma, load_ma, temp_c, values_b) = read_all(&mut adc_a, &mut adc_b);

        //hprintln!("bat_mv {:4}mV bat_ma {:4}mA  load_ma {:5}mA temp_c {}   values_b {:?}", bat_mv, bat_ma, load_ma, temp_c, values_b).unwrap();

        show_display(bat_mv, bat_ma, load_ma, temp_c, values_b, text_style, &mut display);

        delay.delay_ms(2000); // sleep for 2s
    }
}
