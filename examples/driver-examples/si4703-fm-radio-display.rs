// see https://github.com/eldruin/driver-examples/issues/2
// and si4703-fm-radio-display.rs is more developed but also not working

//! Seek an FM radio channel when pressing two buttons "Seek down" / "Seek up"
//! using an Si4703 FM radio receiver (turner) and display the channel
//! frequency in an SSD1306 OLED display.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/si4703-fm-radio-receiver-driver-in-rust/
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1.
//!
//! ```
//! BP    <-> Si4703 <-> Display
//! GND   <-> GND    <-> GND
//! +3.3V <-> VCC    <-> VCC
//! PB8   <-> SCLK   <-> SCL
//! PB9   <-> SDIO   <-> SDA
//! PB7   <-> RST
//! PB6   <-> GPIO2
//! PB10                        <-> Seek up button   <-> +3.3V
//! PB11                        <-> Seek down button <-> +3.3V
//! ```
//!
//! Run with:
//! `cargo embed --example si4703-fm-radio-bp`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};


#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use panic_rtt_target as _;  THIS CAUSE LINK PROBLEM ... undefined symbol: _SEGGER_RTT

//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::dp::{Peripherals};

use rust_integration_testing_of_examples::i2c_led_delay_buttons_stcint::{
                 setup_i2c_led_delay_buttons_stcint_using_dp,  LED,  DelayNs,
                 SEEK, ChannelSpacing, 
                 DeEmphasis, SeekDirection, SeekMode, Si4703, Volume, ErrorWithPin};



#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("Si4703 example");
    hprintln!("Si4703 example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2c, mut led, mut delay, mut buttons, stcint) = setup_i2c_led_delay_buttons_stcint_using_dp(dp);

    hprintln!("manage").unwrap();
    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();

    hprintln!("text_style").unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut buffer: heapless::String<64> = heapless::String::new();
    buffer.clear();
    write!(buffer, "Radio init...").unwrap();
    display.clear_buffer();
    Text::with_baseline(&buffer, Point::zero(), text_style, Baseline::Top,).draw(&mut display).unwrap();
    display.flush().unwrap();
 
    let mut radio = Si4703::new(manager.acquire_i2c());
    radio.enable_oscillator().unwrap();
    delay.delay_ms(500);
    radio.enable().unwrap();
    delay.delay_ms(110);

    radio.set_volume(Volume::Dbfsm28).unwrap();
    radio.set_deemphasis(DeEmphasis::Us50).unwrap();
    radio.set_channel_spacing(ChannelSpacing::Khz100).unwrap();
    radio.unmute().unwrap();

    hprintln!("loop").unwrap();
    write!(buffer, "\nloop...").unwrap();
    display.clear_buffer();
    Text::with_baseline(&buffer, Point::zero(), text_style, Baseline::Top,).draw(&mut display).unwrap();
    display.flush().unwrap();

    loop {
        // Blink LED to indicate looping.
        led.blink(5000_u16, &mut delay);

        let should_seek_down = buttons.seekdown();
        let should_seek_up = buttons.seekup();
        if should_seek_down || should_seek_up {
            // Blink LED every time a new seek is started
            led.blink(50_u16, &mut delay);
            led.blink(50_u16, &mut delay);
            buffer.clear();
            hprintln!("Seeking...").unwrap();
            write!(buffer, "\nSeeking...").unwrap();

            display.clear_buffer();
            Text::with_baseline(&buffer, Point::zero(), text_style, Baseline::Top,)
                .draw(&mut display)
                .unwrap();
            display.flush().unwrap();

            let direction = if should_seek_down {
                SeekDirection::Down
            } else {
                SeekDirection::Up
            };

            buffer.clear();
            loop {
                match radio.seek_with_stc_int_pin(SeekMode::Wrap, direction, &stcint) {
                    Err(nb::Error::WouldBlock) => {hprintln!("x").unwrap()}
                    Err(nb::Error::Other(ErrorWithPin::SeekFailed)) => {
                        write!(buffer, "Seek Failed!  ").unwrap();
                        break;
                    }
                    Err(_) => {
                        write!(buffer, "Error!     ").unwrap();
                        break;
                    }
                    Ok(_) => {
                        let channel = radio.channel().unwrap_or(-1.0);
                        write!(buffer, "Found {:1} MHz ", channel).unwrap();
                        break;
                    }
                }
                hprintln!(".").unwrap();
            }
            display.clear_buffer();
            Text::with_baseline(&buffer, Point::zero(), text_style, Baseline::Top,)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();
        }
    }
}
