//    //////////////////////////////////////////
//        f0, f1, f4,       stm32g4
//   sck     on PA5
//  miso     on PA6
//  mosi     on PA7

//  CsPin    on PA1
//  BusyPin  on PB8 DIO0
//  ReadyPin on PB9 DIO1
//  ResetPin on PA0

//  tx  GPS  on PA2   to  GPS rx
//  rx  GPS  on PA3  from GPS tx

//  scl      on PB10           PA13
//  sda      on PB11           PA14

//  led      on PC13

//    //////////////////////////////////////////

//pub type LoraSpiType =     Sx127x<Base<SpiType, 
//                                  Pin<'A', 1, Output>, Pin<'B', 4>, Pin<'B', 5>, Pin<'A', 0, Output>, 
//                                  Delay<TIM5, 1000000>>>;

// LoraSpiType   might be something like
// impl DelayNs
//    + Transmit<Error = sx127xError<Error>>
//    + Receive<Info = PacketInfo, Error = sx127xError<Error>>,

//    //////////////////////////////////////////


#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use embedded_hal::delay::DelayNs;
pub use embedded_hal::digital::OutputPin;

pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{gpioa::PA8, Output, OpenDrain},
      //prelude::*,
};



// MODE needs the old version as it is passed to the device hal crates
use embedded_hal::spi::{Mode, Phase, Polarity};

use radio_sx127x::{
   // base::Base,
    device::lora::{
        Bandwidth, CodingRate, FrequencyHopping, LoRaChannel, LoRaConfig, PayloadCrc,
        PayloadLength, SpreadingFactor,
    },
    device::{Channel, Modem, PaConfig, PaSelect},
    //Error as sx127xError, // Error name conflict with hals
   // prelude::*, // prelude has Sx127x,
};

// trait needs to be in scope to find  methods start_transmit and check_transmit.
pub use radio::{Receive, Transmit};

// lora and radio parameters

pub const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub const FREQUENCY: u32 = 907_400_000; // frequency in hertz ch_12: 915_000_000, ch_2: 907_400_000

pub const CONFIG_CH: LoRaChannel = LoRaChannel {
    freq: FREQUENCY as u32, // frequency in hertz
    bw: Bandwidth::Bw125kHz,
    sf: SpreadingFactor::Sf7,
    cr: CodingRate::Cr4_8,
};

pub const CONFIG_LORA: LoRaConfig = LoRaConfig {
    preamble_len: 0x8,
    symbol_timeout: 0x64,
    payload_len: PayloadLength::Variable,
    payload_crc: PayloadCrc::Enabled,
    frequency_hop: FrequencyHopping::Disabled,
    invert_iq: false,
};

//   compare other settings in python version
//    lora.set_mode(sx127x_lora::RadioMode::Stdby).unwrap();
//    set_tx_power(level, output_pin) level >17 => PA_BOOST.
//    lora.set_tx_power(17,1).unwrap();
//    lora.set_tx_power(15,1).unwrap();

//baud = 1000000 is this needed for spi or just USART ?

pub const CONFIG_PA: PaConfig = PaConfig {
    output: PaSelect::Boost,
    power: 10,
};

//let CONFIG_RADIO = Config::default() ;

pub const CONFIG_RADIO: radio_sx127x::device::Config = radio_sx127x::device::Config {
    modem: Modem::LoRa(CONFIG_LORA),
    channel: Channel::LoRa(CONFIG_CH),
    pa_config: CONFIG_PA,
    xtal_freq: 32000000, // CHECK
    timeout_ms: 100,
};

