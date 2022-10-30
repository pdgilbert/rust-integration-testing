//! Note that pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.

// Typical use needs other gpio pins for other things (eg cs in digi_pot) and that requires move.
// The move problem needs to be resolved to get this to work



#[cfg(feature = "stm32f4xx")]
// eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{
    gpio::{Alternate, Pin,
           gpioa::{Parts},
    },
    rcc::Clocks,
    pac::{SPI1},
    prelude::*,
    spi::{Spi, TransferModeNormal},
};

//pub type SpiType = Spi<SPI1, impl Pins<SPI1>, TransferModeNormal>;

#[cfg(feature = "stm32f4xx")]
pub type SpiType = Spi<SPI1, (
           Pin<'A', 5_u8, Alternate<5_u8>>, 
           Pin<'A', 6_u8, Alternate<5_u8>>, 
           Pin<'A', 7_u8, Alternate<5_u8>>), TransferModeNormal>;

#[cfg(feature = "stm32f4xx")]
pub fn setup_spi(spix: SPI1, gpioa: Parts, &clocks: &Clocks) -> SpiType {
    let spi = Spi::new(
        spix,
        (
            gpioa.pa5.into_alternate(), // sck   on PA5
            gpioa.pa6.into_alternate(), // miso  on PA6
            gpioa.pa7.into_alternate(), // mosi  on PA7
        ),
        mcp4x::MODE,
        8.MHz(),
        &clocks,
    );

    spi
}

