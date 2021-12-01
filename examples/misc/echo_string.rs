//! Serial DMA RX transfer. Read 15 chars input from console on USART1, echo back to console,
//!  and output to semihost. Repeat.
//!
//! See examples/serial_char.rs for notes about connecting usart1 to
//!   serial ttl-usb converter on computer for console output.
//! That file also has more notes regarding setup.

// Outside of loop this repeat read buf and echo would work, even multple times:
//   let (buf, rx1_ch, rx1) = rx1.read_exact(buf, rx1_ch).wait();
//   let (buf, tx1_ch, tx1) = tx1.write_all( buf, tx1_ch).wait();
// but  when moved into a loop there are problems with
// value moved in previous iteration of loop.
// That problem is fixed by modifying mut tuple but because of
// "cannot ... destructuring assignments" problem the 3-tuple needs to be kept
// and elements referenced.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m::singleton;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
//use nb::block;

// possibly should be using something from embedded-hal here

const BUFSIZE: usize = 15;

pub trait ReadDma {
    // might be nice if this returned buf rather than leaving it as element of self
    fn read(&mut self) -> ();
}

pub trait WriteDma {
    // might be nice if this took buf as argument rather than needing it as element of self
    fn write(&self) -> ();
}

//pub struct RxDma <B, T, U> {
//    buf : B,    //&'static mut [u8; BUFSIZE],
//    ch  : T,
//    rx  : U,
//    }
//
//pub struct TxDma <B, T, U> {
//    buf : B,    //&'static mut [u8; BUFSIZE],
//    ch  : T,
//    tx  : U,
//    }

pub struct RxDma<B, T, U> {
    tup: (B, T, U),
}

pub struct TxDma<B, T, U> {
    tup: (B, T, U),
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg blue pill stm32f103
use stm32f0xx_hal::{
    dma::{dma1::C5, RxDma}, //TxDma,  C4,
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Rx, Serial, Tx},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);

    let (tx, rx) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa9.into_alternate_af1(cs),  //tx pa9
            gpioa.pa10.into_alternate_af1(cs), //rx pa10
        )
    });

    let txrx1 = Serial::usart1(p.USART1, (tx, rx), 9600.bps(), &mut rcc);

    let (tx1, rx1) = txrx1.split();
    let channels = p.DMA1.split(&mut rcc.ahb);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    device::USART1,
    dma::dma1,
    pac::Peripherals,
    prelude::*,
    serial::{Config, Rx, Serial, StopBits, Tx},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut afio = p.AFIO.constrain();
    let mut gpioa = p.GPIOA.split();

    let txrx1 = Serial::usart1(
        p.USART1,
        (
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh), //tx pa9,
            gpioa.pa10,
        ), //rx pa10
        &mut afio.mapr,
        Config::default()
            .baudrate(9600.bps())
            .stopbits(StopBits::STOP1),
        clocks,
    ); //.split();

    let (tx1, rx1) = txrx1.split();

    // Note: in stm32f1xx_hal writeln! does not work with TxDma
    //  writeln!(tx1, "\r\ncheck console output.\r\n").unwrap();
    //  and without dma tx1.write() expects u8 not &[u8; 25]

    let dma1 = p.DMA1.split();
    let (tx1_ch, rx1_ch) = (dma1.4, dma1.5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    dma::dma1,
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1, impl TxPin<USART1>>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1, impl RxPin<USART1>>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);

    let txrx1 = Serial::new(
        p.USART1,
        (
            gpioa
                .pa9
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        9600.Bd(),
        clocks,
        &mut rcc.apb2,
    );

    let (tx1, rx1) = txrx1.split();

    let dma1 = p.DMA1.split(&mut rcc.ahb);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);
    //let (tx2_ch, rx2_ch) = (dma1.ch6, dma1.ch7);
    //let (tx3_ch, rx3_ch) = (dma1.ch3, dma1.ch2);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    //impl WriteDma for TxDma<&'static mut [u8; 15], dma1::C4,  Tx<USART1>> {
    //  fn write(&self)  -> () {
    //     let tx = self.tup.2;
    //     tx.write_all(&self.tup.0, self.tup.1).wait();   //sending self buffer tup.0
    //     //^^^^^^^^^ expected `&mut [u8; 15]`, found `u8`
    //     //         ^^^^^^^^^^^ the trait `embedded_dma::Word` is not implemented for `&mut [u8; 15]`
    //     // ... requirements on the impl of `embedded_dma::ReadTarget` for `&mut [u8; 15]`
    //     // ... requirements on the impl of `embedded_dma::ReadBuffer` for `&&mut [u8; 15]`
    //     }
    //  };
    //
    //impl ReadDma for RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>> {
    //  fn read(&mut self)  ->  () {
    //     let rx = self.tup.2;
    //     rx.read_exact(self.tup.0, self.tup.1).wait();   //result in self buffer tup.0
    //     }
    //  };

    //       let  send = TxDma{buf: txbuf,  ch: tx1_ch,  tx:tx1};
    //       let  recv = RxDma{buf: rxbuf,  ch: rx1_ch,  rx:rx1};

    //       impl WriteDma for TxDma<&'static mut [u8; 15], dma1::C4,  Tx<USART1>> {
    //           fn write(&self)  -> () {
    //              self.tx.write_all(&self.buf, self.ch).wait();
    //            }
    //           };
    //
    //       impl ReadDma for RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>> {
    //           fn read(&mut self)  ->  () {
    //              self.rx.read_exact(self.buf, self.ch).wait();   //result in self.buf
    //            }
    //           };

    (send, recv)
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    pac::dma1,
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split();
    //p.USART1.cr1.modify(|_,w| w.rxneie().set_bit());  //need RX interrupt?

    let txrx1 = Serial::new(
        p.USART1,
        (gpioa.pa9.into_alternate(), gpioa.pa10.into_alternate()),
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap();

    let (mut tx1, mut rx1) = txrx1.split();

    let dma1 = p.DMA1.split(&mut rcc.cfgr);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split();
    p.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?

    let txrx1 = Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate(),
            gpioa.pa10.into_alternate(),
        ),
        Config::default().baudrate(9600.Bps()),
        clocks,
    );

    let (mut tx1, mut rx1) = txrx1.split();

    let dma1 = p.DMA1.split(&mut rcc.cfgr);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Rx, Tx},
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;
    let gpioa = p.GPIOA.split(ccdr.peripheral.GPIOA);

    let (mut tx1, mut rx1) = p
        .USART1
        .serial(
            (
                gpioa.pa9.into_alternate_af7(), //tx pa9
                gpioa.pa10.into_alternate_af7(),
            ), //rx pa10
            9600.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split();

    let dma1 = p.DMA1.split(&mut rcc.cfgr);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
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
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let gpioa = p.GPIOA.split(&mut rcc);

    let (mut tx1, mut rx1) = p
        .USART1
        .usart(
            gpioa.pa9,  //tx pa9
            gpioa.pa10, //rx pa10
            Config::default(),
            &mut rcc,
        )
        .unwrap()
        .split();

    let dma1 = p.DMA1.split(&mut rcc.cfgr);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    prelude::*,
    rcc, // for ::Config but note name conflict with next
    serial::{Config, Rx, SerialExt, Tx},
    stm32::Peripherals,
    stm32::USART1,
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], dma1::C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], dma1::C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split(&mut rcc);
    p.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let txrx1 = p.USART1.usart(
        (
            gpioa.pa9, //tx pa9
            gpioa.pa10,
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        &mut rcc,
    ).unwrap();

    let (mut tx1, mut rx1) = txrx1.split();

    let dma1 = p.DMA1.split(&mut rcc.cfgr);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    //pac::dma1,
    dma::dma1::{C4, C5},
    pac::Peripherals,
    pac::USART1,
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    TxDma<&'static mut [u8; 15], C4, Tx<USART1>>,
    RxDma<&'static mut [u8; 15], C5, Rx<USART1>>,
) {
    //fn setup() ->  (impl WriteDma, impl ReadDma) {

    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    let txrx1 = Serial::usart1(
        p.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    );

    let (tx1, rx1) = txrx1.split();

    let dma1 = p.DMA1.split(&mut rcc.ahb1);
    let (tx1_ch, rx1_ch) = (dma1.4, dma1.5);

    let txbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters
    let rxbuf = singleton!(: [u8; BUFSIZE] = *b"---- empty ----").unwrap(); //NB. 15 characters

    let send = TxDma {
        tup: (txbuf, tx1_ch, tx1),
    };
    let recv = RxDma {
        tup: (rxbuf, rx1_ch, rx1),
    };

    (send, recv)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    //see serial_char.rs and  echo_by_char.rs for additional comments.

    let (mut send, mut recv) = setup();

    hprintln!("test write to console ...").unwrap();

    //    *send.tup.0 = *b"\r\nSlowly type  ";  //NB. 15 characters
    //    send.tup = send.tup.2.write_all( send.tup.0, send.tup.1).wait();

    //    *send.buf = *b"\r\nSlowly type  ";  //NB. 15 characters
    //    send.buf = send.tx.write_all( send.buf, send.ch).wait().0;
    //    send.write();

    // Now read from console into  buf and echo back to console

    hprintln!("Enter 15 characters in console. Repeat.").unwrap();
    hprintln!("Use ^C in gdb to exit.").unwrap();

    // Note send (write) is using buf as put into recv (read). The returned buffer in recv and
    //   the argument buffer in send are data. The argument buffer in recv may be a holding spot
    //   to put return buffer? but it is not part of the program logic. The size of the return
    //   buffer from recv does seem to be determined by the size of the recv argument buffer.
    //   The return buffer from send seems like it should be unnecessary, but it does provide
    //   the buffer needed in the recv argument.

    //each pass in loop waits for input of 15 chars typed in console then echos them
    loop {
        // using struct  with tuple (buf, ch, tx) builds with stm32f3xx_hal:

        recv.tup = recv.tup.2.read_exact(send.tup.0, recv.tup.1).wait();
        send.tup = send.tup.2.write_all(recv.tup.0, send.tup.1).wait();

        // but this will not work with other hals because .read_exact() and .write_all()
        // are specific to stm32f3xx_hal. A generic API across hals would be nice.
        // Getting traits for read and write to work would as provide a way
        // to work around the hal differences.

        //       recv.tup = recv.tup.2.read().unwrap();
        //       send.tup = send.tup.2.write(recv.tup.0).unwrap();

        // using struct with elements buf, ch, tx does not build with stm32f3xx_hal:
        //  (problems with modified in previous iteration of loop, amoung others)
        //       recv.buf = recv.rx.read_exact(send.buf, recv.ch).wait().0;
        //       send.buf = send.tx.write_all( recv.buf, send.ch).wait().0;

        // using traits for read and write does not build with stm32f3xx_hal:
        // ( cannot seem to get working impl of traits, and then need to move read buf to write)
        //       recv.read();
        //       //send.buf = recv.buf;
        //       send.write();
    }
}
