// Code here is based on https://github.com/eldruin/driver-examples/

//! Seek an FM radio channel when pressing two buttons "Seek down" / "Seek up"
//! using an Si4703 FM radio receiver (turner).
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/si4703-fm-radio-receiver-driver-in-rust/
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1.
//!
//! ```
//! BP    <-> Si4703
//! GND   <-> GND
//! +3.3V <-> VCC
//! PB8   <-> SCLK
//! PB9   <-> SDIO
//! PB7   <-> RST
//! PB6   <-> GPIO2
//! PB10             <-> Seek up button   <-> +3.3V
//! PB11             <-> Seek down button <-> +3.3V
//! ```
//!
//! Run with:
//! `cargo embed --example si4703-fm-radio-bp`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nb::block;
//use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use rust_integration_testing_of_examples::dp::{Peripherals};

use rust_integration_testing_of_examples::i2c_led_delay_buttons_stcint::{
                 setup_i2c_led_delay_buttons_stcint_using_dp,  LED,  DelayUs,
                 SEEK, ChannelSpacing, 
                 DeEmphasis, SeekDirection, SeekMode, Si4703, Volume};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Si4703 example");
    let dp = Peripherals::take().unwrap();

    let (i2c, mut led, mut delay, mut buttons, stcint) = setup_i2c_led_delay_buttons_stcint_using_dp(dp);

    //    let mut rst = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
    //    let stcint = gpiob.pb6.into_pull_up_input(&mut gpiob.crl);
    //    let seekdown = gpiob.pb11.into_pull_down_input(&mut gpiob.crh);
    //    let seekup = gpiob.pb10.into_pull_down_input(&mut gpiob.crh);
    //    reset_si4703(&mut rst, &mut sda, &mut delay).unwrap();

    let manager = shared_bus::BusManagerSimple::new(i2c);

    let mut radio = Si4703::new(manager.acquire_i2c());
    radio.enable_oscillator().unwrap();
    delay.delay_ms(500);
    radio.enable().unwrap();
    delay.delay_ms(110);

    radio.set_volume(Volume::Dbfsm28).unwrap();
    radio.set_deemphasis(DeEmphasis::Us50).unwrap();
    radio.set_channel_spacing(ChannelSpacing::Khz100).unwrap();
    radio.unmute().unwrap();
    loop {
        // Blink LED 0 every time a new seek is started
        // to check that everything is actually running.
        led.blink(50_u16, &mut delay);

        let should_seek_down = buttons.seekdown();
        let should_seek_up = buttons.seekup();
        if should_seek_down || should_seek_up {
            let direction = if should_seek_down {
                SeekDirection::Down
            } else {
                SeekDirection::Up
            };

            block!(radio.seek_with_stc_int_pin(SeekMode::Wrap, direction, &stcint)).unwrap();
        }
    }
}
