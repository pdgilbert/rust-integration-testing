//! Note that i2c pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.


// "hal" is used for items that are the same in all hal  crates
use crate::stm32xxx_as_hal::hal;

use hal::{
    pac::{I2C1, I2C2},
    i2c::I2c as I2cType,
  //  i2c::Error as i2cError,
    prelude::*,
};

#[cfg(not(feature = "stm32g4xx"))] 
pub type I2c1Type = I2cType<I2C1>;

#[cfg(not(feature = "stm32g4xx"))] 
pub type I2c2Type = I2cType<I2C2>;



#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    gpio::{Alternate, AF1,
           gpiob::{PB7, PB8, PB10, PB11, Parts}, 
    },
    rcc::Rcc,
};


//#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
//pub type I2c1Type = I2c<I2C1, PB8<Alternate<AF1>>, PB7<Alternate<AF1>>>;
//pub type I2c1Type = I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,


#[cfg(feature = "stm32f0xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: Parts, rcc: &mut Rcc) -> I2c1Type {
    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (gpiob.pb8.into_alternate_af1(cs), // scl on PB8
         gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });
    let i2c = I2c::i2c1(i2c1, (scl, sda), 400.khz(), rcc);

    i2c
}


//#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
//pub type I2c2Type = I2c<I2C2, PB10<Alternate<AF1>>, PB11<Alternate<AF1>>>;
//
//#[cfg(feature = "stm32f0xx")]
//pub fn setup_i2c2(i2c2: I2C2, gpiob: Parts, rcc: &mut Rcc) -> I2c2Type {
//    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
//        (gpiob.pb10.into_alternate_af1(cs),
//         gpiob.pb11.into_alternate_af1(cs),
//        )
//    });
//    let i2c = I2c::i2c2(i2c2, (scl, sda), 400.khz(), rcc);
//
//    i2c
//}

#[cfg(feature = "stm32f0xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpiob: Parts, rcc: &mut Rcc) -> (I2c1Type, I2c2Type) {
    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (gpiob.pb8.into_alternate_af1(cs), // scl on PB8
         gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });
    let i2c1 = I2c::i2c1(i2c1, (scl, sda), 400.khz(), rcc);

    let (scl, sda) = cortex_m::interrupt::free(move |cs| {
        (gpiob.pb10.into_alternate_af1(cs),
         gpiob.pb11.into_alternate_af1(cs),
        )
    });
    let i2c2 = I2c::i2c2(i2c2, (scl, sda), 400.khz(), rcc);

    (i2c1, i2c2)
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode},   
    gpio::{gpiob::{PB8, PB9, PB10, PB11, Parts}, Alternate, OpenDrain},
    afio::Parts as afioParts,
    rcc::Clocks,
};


//#[cfg(feature = "stm32f1xx")]
//pub type I2c1Type = BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;   
// this works in a function signature BlockingI2c<I2C1, impl Pins<I2C1>>;

#[cfg(feature = "stm32f1xx")]
pub fn setup_i2c1(i2c1: I2C1, mut gpiob: Parts, afio: &mut afioParts, &clocks: &Clocks) -> I2c1Type {
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 100_000_u32.Hz(),
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


// It would be nicer if setup_i2c1 and then setup_i2c2 could be called, then
// setup_i2c1_i2c2 could be removed. But there is a problem with partial move of dp or of gpiob.
// A method like dp.I2C1.config() or .into()  might help?

#[cfg(feature = "stm32f1xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2 , mut gpiob: Parts, afio: &mut afioParts, &clocks: &Clocks) -> (I2c1Type, I2c2Type) {
    let i2c1 = BlockingI2c::i2c1(
        i2c1,
        (gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh), 
         gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh)
        ),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 100_000_u32.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let i2c2 = BlockingI2c::i2c2(
        i2c2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 (PB8, PB9) but //NOT i2c2
        Mode::Fast {
            frequency: 400_000_u32.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    (i2c1, i2c2)
}



#[cfg(feature = "stm32f3xx")]
use stm32f3xx_hal::{
    gpio::{OpenDrain, AF4,
           gpioa::{PA9, PA10, Parts as PartsA}, 
           gpiob::{PB6, PB7,  Parts as PartsB}
    },
    rcc::{Clocks, APB1},
};

//#[cfg(feature = "stm32f3xx")]
//pub type I2c1Type = I2c<I2C1, (PB6<AF4<OpenDrain>>, PB7<AF4<OpenDrain>>)>;
//pub type I2c1Type = I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)> ;

#[cfg(feature = "stm32f3xx")]
pub fn setup_i2c1(i2c1: I2C1, mut gpiob: PartsB, clocks: Clocks, mut apb1: APB1) -> I2c1Type {
    let scl = gpiob.pb6.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    //    // //NOT sure if pull up is needed
    //    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c = I2c::new(i2c1, (scl, sda), 100_000.Hz(), clocks, &mut apb1);

    i2c
}


#[cfg(feature = "stm32f3xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2,  mut gpioa: PartsA, mut gpiob: PartsB, clocks: Clocks, mut apb1: APB1) -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb6.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    //    // //NOT sure if pull up is needed
    //    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c1 = I2c::new(i2c1, (scl, sda), 100_000.Hz(), clocks, &mut apb1);

    let scl =  gpioa.pa9.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let sda = gpioa.pa10.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    //    // //NOT sure if pull up is needed
    //    scl.internal_pull_up(&mut gpiob.pupdr, true);
    //    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c2 = I2c::new(i2c2, (scl, sda), 100_000.Hz(), clocks, &mut apb1);

    (i2c1, i2c2)
}


#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    //gpio::{Alternate, OpenDrain, gpiob::{PB8, PB9,  PB10, PB3, Parts as PartsB}},
    gpio::{gpiob::{Parts as PartsB}},
    rcc::Clocks,   
};

#[cfg(feature = "stm32f4xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, &clocks: &Clocks) -> I2c1Type {
    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); 

    let i2c = I2cType::new(i2c1, (scl, sda), 400.kHz(), &clocks);
    i2c
}


#[cfg(feature = "stm32f4xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpiob: PartsB, &clocks: &Clocks ) -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); 
    let i2c1 = I2cType::new(i2c1, (scl, sda), 400.kHz(), &clocks);

    let scl = gpiob.pb10.into_alternate_open_drain(); // scl on PB10
    let sda = gpiob.pb3.into_alternate_open_drain(); // sda on PB3
    let i2c2 = I2cType::new(i2c2, (scl, sda), 400.kHz(), &clocks);

    (i2c1, i2c2)
}


#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    gpio::{Alternate, OpenDrain,
           gpiob::{PB8, PB9, PB10, PB11, Parts as PartsB},
    },
    i2c::{BlockingI2c, Mode, },
    rcc::{Clocks, APB1},
    
    prelude::*,
};

//#[cfg(feature = "stm32f7xx")]
//pub type I2c1Type = BlockingI2c<I2C1, PB8<Alternate<4u8, OpenDrain>>, PB9<Alternate<4u8, OpenDrain>>>;
//pub type I2c1Type =  BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>;

#[cfg(feature = "stm32f7xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, &clocks: &Clocks, mut apb1: &mut APB1) -> I2c1Type {

    let scl = gpiob.pb8.into_alternate_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_open_drain(); // sda on PB9

    let i2c = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

    i2c
}


#[cfg(feature = "stm32f7xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpiob: PartsB, &clocks: &Clocks, mut apb1: &mut APB1) -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb8.into_alternate_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_open_drain(); // sda on PB9

    let i2c1 = BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

    let scl = gpiob.pb10.into_alternate_open_drain(); 
    let sda = gpiob.pb11.into_alternate_open_drain(); 

    let i2c2 = BlockingI2c::i2c2(
        i2c2,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut apb1,
        1000,
    );

    (i2c1, i2c2)
}


#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    i2c::{Config as i2cConfig,},
    gpio::{Output, OpenDrain, gpiob::{PB8, PB9,  PB10, PB11, Parts as PartsB}
    },
    rcc::Rcc,
};

#[cfg(feature = "stm32g0xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, rcc: &mut Rcc) -> I2c1Type {
    let scl = gpiob.pb8.into_open_drain_output_in_state(PinState::High);
    let sda = gpiob.pb9.into_open_drain_output_in_state(PinState::High); 
    
    let i2c = I2c::i2c1(i2c1,  sda, scl,  i2cConfig::with_timing(0x2020_151b), rcc);
    i2c
}

#[cfg(feature = "stm32g0xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpiob: PartsB, rcc: &mut Rcc) -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb8.into_open_drain_output_in_state(PinState::High); 
    let sda = gpiob.pb9.into_open_drain_output_in_state(PinState::High); 
    let i2c1 = I2c::i2c1(i2c1,  sda, scl,  i2cConfig::with_timing(0x2020_151b), rcc);

    let scl = gpiob.pb10.into_open_drain_output_in_state(PinState::High);
    let sda = gpiob.pb11.into_open_drain_output_in_state(PinState::High); 
    let i2c2 = I2c::i2c2(i2c2,  sda, scl,  i2cConfig::with_timing(0x2020_151b), rcc);

    (i2c1, i2c2)
}

#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    i2c::{Config},// SDAPin, SCLPin},
    gpio::{AlternateOD,
           gpioa::{PA8, PA9, Parts as PartsA},
           gpiob::{PB8, PB9, Parts as PartsB}
    },
    rcc::Rcc,
    time::{RateExtU32},
};

#[cfg(feature = "stm32g4xx")]
pub type I2c1Type = I2cType<I2C1, PB9<AlternateOD<4_u8>>, PB8<AlternateOD<4_u8>>>;
//pub type I2c1Type = I2c<I2C1, impl SCLPin<I2C1>, impl SDAPin<I2C1>>;

#[cfg(feature = "stm32g4xx")]
pub type I2c2Type = I2cType<I2C2, PA8<AlternateOD<4_u8>>, PA9<AlternateOD<4_u8>>>;
//pub type I2c2Type = I2c<I2C2, impl SCLPin<I2C2>, impl SDAPin<I2C2>>;

#[cfg(feature = "stm32g4xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, rcc: &mut Rcc) -> I2c1Type {
//pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, rcc: &mut Rcc) -> I2c<I2C1, impl SCLPin<I2C1>, impl SDAPin<I2C1>> {
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); 

    //let i2c = I2c::new(i2c1, (scl, sda), 400.kHz(), &clocks);
    let i2c = i2c1.i2c(sda, scl, Config::new(400.kHz()), rcc);

    i2c
}


#[cfg(feature = "stm32g4xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpioa: PartsA, gpiob: PartsB, rcc: &mut Rcc )
       -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); 
    let i2c1 = i2c1.i2c(sda, scl, Config::new(400.kHz()), rcc);

    let scl = gpioa.pa9.into_alternate_open_drain();
    let sda = gpioa.pa8.into_alternate_open_drain(); 
    let i2c2 = i2c2.i2c(sda, scl, Config::new(400.kHz()), rcc);

    (i2c1, i2c2)
}


#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    gpio::{gpiob::Parts as PartsB,
           gpiof::Parts as PartsF},
    rcc::{CoreClocks, rec::I2c1, rec::I2c2},
};

#[cfg(feature = "stm32h7xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, i2cx1: I2c1, 
                  &clocks: &CoreClocks) -> I2c1Type {
    
    i2c1.i2c((gpiob.pb8.into_alternate().set_open_drain(), // scl  
              gpiob.pb9.into_alternate().set_open_drain(), // sda
             ), 400.kHz(), i2cx1, &clocks)
}

#[cfg(feature = "stm32h7xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, gpiob: PartsB, i2cx1: I2c1,
                       i2c2: I2C2, gpiof: PartsF, i2cx2: I2c2, 
                       &clocks: &CoreClocks) ->  (I2c1Type, I2c2Type) {

   (
    i2c1.i2c((gpiob.pb8.into_alternate().set_open_drain(), // scl  
              gpiob.pb9.into_alternate().set_open_drain(), // sda
             ), 400.kHz(), i2cx1, &clocks),

    i2c2.i2c((gpiof.pf1.into_alternate().set_open_drain(), // scl
              gpiof.pf0.into_alternate().set_open_drain(), // sda
             ), 400.kHz(), i2cx2, &clocks)
   )
}


#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    gpio::{
        gpiob::{PB8, PB9, PB10, PB11, Parts as PartsB},
        OpenDrain, Output, 
    },
    rcc::Rcc,
};

#[cfg(feature = "stm32l0xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, mut rcc: Rcc) -> I2c1Type {

    // could also have scl on PB6, sda on PB7
    //BlockingI2c::i2c1(
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c = i2c1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    i2c
}


#[cfg(feature = "stm32l0xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpiob: PartsB, mut rcc: Rcc) -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c1 = i2c1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    let scl = gpiob.pb10.into_open_drain_output(); 
    let sda = gpiob.pb11.into_open_drain_output();
    let i2c2 = i2c2.i2c(sda, scl, 400_000.Hz(), &mut rcc);

   (i2c1, i2c2)
}


#[cfg(feature = "stm32l1xx")]
use stm32l1xx_hal::{
    gpio::{Output, OpenDrain, 
           gpiob::{PB8, PB9, PB10, PB11, Parts as PartsB}, 
    },
    rcc::Rcc,
};

//#[cfg(feature = "stm32l1xx")]
//pub type I2c1Type = I2c<I2C1, (PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>)>;

#[cfg(feature = "stm32l1xx")]
pub fn setup_i2c1(i2c1: I2C1, gpiob: PartsB, mut rcc: Rcc) -> I2c1Type {
    // NOTE THIS SETUP FUNCTION IS sensitive to placement because it consumes rcc??
    // All pin possibities are gpiob:  PB10-11 on I2C2,  PB6-7 on I2C1, PB8-9 on I2C1, 
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c = i2c1.i2c((scl, sda), 400.khz(), &mut rcc);

    i2c
}


#[cfg(feature = "stm32l1xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, gpiob: PartsB, mut rcc: Rcc) -> (I2c1Type, I2c2Type) {
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c1 = i2c1.i2c((scl, sda), 400.khz(), &mut rcc);

    let scl = gpiob.pb10.into_open_drain_output();
    let sda = gpiob.pb11.into_open_drain_output();
    let i2c2 = i2c2.i2c((scl, sda), 400.khz(), &mut rcc);

    (i2c1, i2c2)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    gpio::{OpenDrain, Alternate,
           gpiob::{PB8, PB9, PB10, PB11, Parts},
    },
    i2c::{Config},
    rcc::{Clocks, APB1R1},
};

//#[cfg(feature = "stm32l4xx")]
//pub type I2c1Type = I2c<I2C1, (PB8<Alternate<OpenDrain, 4u8>>, PB9<Alternate<OpenDrain, 4u8>>)>;
//pub type I2c1Type =  I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>;

#[cfg(feature = "stm32l4xx")]
pub fn setup_i2c1(i2c1: I2C1, mut gpiob: Parts, &clocks: &Clocks, apb1r1: &mut APB1R1) -> I2c1Type {
    let mut scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let mut sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = I2c::i2c1(i2c1, (scl, sda), Config::new(400_u32.kHz(), clocks), apb1r1 );

    i2c
}


#[cfg(feature = "stm32l4xx")]
pub fn setup_i2c1_i2c2(i2c1: I2C1, i2c2: I2C2, mut gpiob: Parts, &clocks: &Clocks, apb1r1: &mut APB1R1) -> (I2c1Type, I2c2Type) {
    let mut scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let mut sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c1 = I2c::i2c1(i2c1, (scl, sda), Config::new(400_u32.kHz(), clocks), apb1r1 );

    let mut scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    scl.internal_pull_up(&mut gpiob.pupdr, true);

    let mut sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c2 = I2c::i2c2(i2c2, (scl, sda), Config::new(400_u32.kHz(), clocks),  apb1r1 );

    (i2c1, i2c2)
}
