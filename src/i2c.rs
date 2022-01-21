
#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::{I2C1, I2C2},
    prelude::*,
    gpio::{gpiob::{PB8, PB9, PB10, PB11, Parts}, Alternate, OpenDrain},
    rcc::Clocks,
    afio::Parts as afioParts,
};

#[cfg(feature = "stm32f1xx")]
pub type I2c1Type = BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;   
// this works in a function signature BlockingI2c<I2C1, impl Pins<I2C1>>;

#[cfg(feature = "stm32f1xx")]
pub fn setup_i2c1(i2c1: I2C1, mut gpiob: Parts, mut afio: afioParts, &clocks: &Clocks) -> I2c1Type {

    //let mut afio = dp.AFIO.constrain();
    //let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 100_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    i2c
}

#[cfg(feature = "stm32f1xx")]
pub type I2c2Type = BlockingI2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>;   

#[cfg(feature = "stm32f1xx")]
pub fn setup_i2c2(i2c2: I2C2 , mut gpiob: Parts, &clocks: &Clocks) -> I2c2Type {

    //let mut gpiob = dp.GPIOB.split();

    let i2c = BlockingI2c::i2c2(
        i2c2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 (PB8, PB9) but not i2c2
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    i2c
}
