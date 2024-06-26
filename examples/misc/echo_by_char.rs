//! Echo console input back to console +  rprintln output (previously used semihost hprintln), char by char
//!
//! Connect the Tx pin pa9  to the Rx pin of usb-ttl converter
//! Connect the Rx pin pa10 to the Tx pin of usb-ttl converter
//! Set up the serial console (e.g. minicom) with the same settings used here.
//! (Using 9600bps, could be higher but needs serial console to be the same.)

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;

//use embedded_hal::serial::nb::Write;
//use core::fmt::Write;  // for writeln, but not supported by stm32f3xx_hal
//use embedded_hal::blocking::serial::{Write};  // for write trait
//use embedded_hal::blocking::serial::{Write, Read}; 
//use embedded_hal::serial::{Write, Read};
//use embedded_hal::serial::{Write};

use embedded_io::{Read, Write};

//use core::str::from_utf8;

//use embedded_hal::serial;

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg blue pill stm32f103
use stm32f0xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Rx, Serial, Tx},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);

    let (tx, rx) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa9.into_alternate_af1(cs),  //tx pa9
            gpioa.pa10.into_alternate_af1(cs), //rx pa10
        )
    });

    Serial::usart1(p.USART1, (tx, rx), 9600.bps(), &mut rcc).split()
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    pac::{Peripherals, USART1},
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    //fn setup() -> (impl WriteRead<USART1>) {
    //fn setup() -> (impl Write<USART1>, impl Read<USART1>) {
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut afio = p.AFIO.constrain();
    let mut gpioa = p.GPIOA.split();
    // next consumes (moves) arguments other than clocks,  &mut rcc.apb2 and afio.
    Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh), //tx pa9
            gpioa.pa10,
        ), //rx pa10
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()), //.stopbits(StopBits::STOP1),
        &clocks,
    )
    .split()
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Tx<USART1, impl TxPin<USART1>>,
    Rx<USART1, impl RxPin<USART1>>,
) {
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
    //let cnfg = 9600.bps();
    Serial::new(
        p.USART1,
        (
            gpioa
                .pa9
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        9600.Bd(),
        clocks,
        &mut rcc.apb2,
    )
    .split()
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split();
    //p.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    Serial::new(
        p.USART1,
        (gpioa.pa9.into_alternate(), gpioa.pa10.into_alternate()),
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap()
    .split()
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Config, Oversampling, Rx, Serial, Tx, DataBits, Parity},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let gpioa = p.GPIOA.split();

    let serial1 = Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        &clocks,
        Config {
            baud_rate: 115_200.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
    );
    serial1.split()
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{FullConfig, Tx, Rx},
};

#[cfg(feature = "stm32g0xx")]
fn setup() -> (Tx<USART1, FullConfig>, Rx<USART1, FullConfig>) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa =dp.GPIOA.split(&mut rcc);

    dp.USART1.usart((gpioa.pa9, gpioa.pa10),
                        FullConfig::default(), &mut rcc).unwrap().split()
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    stm32::Peripherals,
    stm32::USART1,
    prelude::*,
    serial::{FullConfig, Rx, Tx, NoDMA},
    gpio::{Alternate, gpioa::{PA9, PA10}},
};

#[cfg(feature = "stm32g4xx")]
fn setup() -> (Tx<USART1, PA9<Alternate<7_u8>>, NoDMA>, Rx<USART1, PA10<Alternate<7_u8>>, NoDMA>) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);

    dp.USART1.usart(
       gpioa.pa9.into_alternate(), 
       gpioa.pa10.into_alternate(), 
       FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split()
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Rx, Tx}, //Serial,
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;
    let gpioa = p.GPIOA.split(ccdr.peripheral.GPIOA);

    //let txrx =Serial::usart1(
    //    p.USART1,
    //    (gpioa.pa9.into_alternate_af7(),                          //tx pa9
    //     gpioa.pa10.into_alternate_af7()),                        //rx pa10
    //    9600.bps(),
    //    &clocks,
    //    ).unwrap().split()

    p.USART1
        .serial(
            (
                gpioa.pa9.into_alternate(), //tx pa9
                gpioa.pa10.into_alternate(),
            ), //rx pa10
            9600.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split()
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, Serial1Ext, Tx},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let gpioa = p.GPIOA.split(&mut rcc);

    p.USART1
        .usart(
            gpioa.pa9,  //tx pa9
            gpioa.pa10, //rx pa10
            Config::default(),
            &mut rcc,
        )
        .unwrap()
        .split()
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, SerialExt, Tx},
    stm32::Peripherals,
    stm32::USART1,
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());
    //let clocks  = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split(&mut rcc);

    // following github.com/stm32-rs/stm32l1xx-hal/blob/master/examples/serial.rs
    // Note that setting the alternate function mode  and push_pull input/output is
    // not necessary. The hal code knows to do this for a usart.
    p.USART1
        .usart(
            (
                gpioa.pa9, //tx pa9
                gpioa.pa10,
            ), //rx pa10
            Config::default().baudrate(9600.bps()),
            &mut rcc,
        )
        .unwrap()
        .split()
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>) {
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    Serial::usart1(
        p.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    )
    .split()
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("echo_by_char example");
    //when trying rtt again see https://github.com/probe-rs/probe-rs/issues/1384

    let (mut tx1, mut rx1) = setup();

    hprintln!("test write to console ...").unwrap();
    //rprintln!("test write to console ...");

    tx1.write(b"\r\nconsole connect check.\r\n").ok();

//    for byte in b"\r\nconsole connect check.\r\n" {
//        #[cfg(not(feature = "stm32h7xx"))]
//        block!(tx1.write(*byte)).ok();
//        #[cfg(feature = "stm32h7xx")]
//        block!(tx1.write_byte(*byte)).ok();
//    }

    hprintln!("test read and write by char. Please type into the console ...").unwrap();

    tx1.write(b"test read and write by char. Please type into the console ...").ok();

    //rprintln!("test read and write by char. Please type into the console ...");
    //writeln!(tx1, "\r\nPlease type (slowly) into the console below:\r\n").unwrap();
//    for byte in b"\r\nType (slowly) below:\r\n" {
//        #[cfg(not(feature = "stm32h7xx"))]
//        block!(tx1.write(*byte)).ok();
//        #[cfg(feature = "stm32h7xx")]
//        block!(tx1.write_byte(*byte)).ok();
//    }

    let mut buffer: [u8; 5] = [0; 5];  // could be length 1 for a byte

    loop {
        // Read a byte and write
        let _len = rx1.read(&mut buffer);

//        #[cfg(not(feature = "stm32h7xx"))]
//        let received = block!(rx1.read()).unwrap();
//        #[cfg(feature = "stm32h7xx")]
//        let received = block!(rx1.read_byte()).unwrap();

        //hprintln!("received").unwrap();  // for debugging

        tx1.write(&buffer).ok();

//        #[cfg(not(feature = "stm32h7xx"))]
//        block!(tx1.write(received)).ok();
//        #[cfg(feature = "stm32h7xx")]
//        block!(tx1.write_byte(received)).ok();

        hprintln!("{:?}", &buffer).unwrap();
        //rprintln!("{}", from_utf8(&[buffer]).unwrap());
    }
}
