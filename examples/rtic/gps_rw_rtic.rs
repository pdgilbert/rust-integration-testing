//!  Tested on weact-stm32g474 July 12, 2024.
//!   Compiles but NOT working. Gives len Err(Overrun). Compare misc/gps_rw.rs which does work.
//!   Also, the rx_gps.read(buf)  handling of buf in rtic seem to require u8 not [u8; 80] ?
//!
//! Two UARTS, no external device crates

//! Serial interface read GPS one usart and write on another usart to USB-TTL console (minicom).
//!
//! usart1 connect the Tx pin (pa9  on bluepill) to the Rx pin of a serial-usb converter
//! usart1 connect the Rx pin (pa10 on bluepill) to the Tx pin of a serial-usb converter
//! Set up the serial console (e.g. minicom) with the same settings used here.
//! (Using 9600bps, could be higher but needs serial console to be the same.)
//!
//! GPS uses 9600bps, 8bit, odd parity, 1 stopbit. This can be confirmed by connecting GPS
//!  directly to the  USB-TTL and terminal with these settings (minicom 8-N-1)
//! The usart and pins for the GPS depend on the board. For specifics see setup_from_dp() sections below.
//!
//! See examples/serial_char.rs for notes about connecting usart1 to
//!   serial-usb converter on computer for console output.
//! That file also has for more notes regarding setup below.

#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;



use rtic::app;
use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000); 

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    //use core::fmt::Write;
    use cortex_m_semihosting::{hprintln};
    
    use rtic;
    use crate::Mono;

    //use nb::block;

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::setup::{MONOCLOCK, Tx1Type, Rx2Type};

    use embedded_io::{Read, Write};

    #[shared]
    struct Shared {
        //led: LedType,
        buffer: [u8; 80],
        //buffer: heapless::Vec<u8, 80>,
    }

    #[local]
    struct Local {
        tx_con: Tx1Type, 
        rx_gps: Rx2Type,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
      
        Mono::start(cx.core.SYST, MONOCLOCK);

        // byte buffer up to 80  u8 elements on stack
        let mut buffer: [u8; 80] = [0; 80];
        //let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
        //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap(); //0 of 80
        //buffer.clear();
        
        // transmit to console, receive from gps
        let (mut tx_con, _rx1, _tx2, rx_gps) = setup::tx1_rx1_tx2_rx2_from_dp(cx.device);

        tx_con.write_all("\r\nconsole connect check.\r\n".as_bytes()).unwrap(); 

        // note that putting hprintln! in loop slows it too much and data is lost.
     
        // read gps on usart2
        hprintln!("starting read GPS").unwrap();
        read_gps::spawn().unwrap();

        (Shared {buffer}, Local {tx_con, rx_gps} )
    }

    #[task(shared = [buffer], local = [rx_gps])]
    async fn read_gps(cx: read_gps::Context) {
        let mut buffer = cx.shared.buffer;
        let rx_gps     = cx.local.rx_gps;
       
        loop {
            //let len = buffer.lock(|buf| {rx_gps.read(buf)});
            let mut bf: [u8; 80] = [0; 80];
            let len = rx_gps.read(&mut bf);
        hprintln!("len {:?}", len).unwrap();  // indicates len Err(Overrun)
            match len {
               Ok(length) => {  if length != 0 {write_con::spawn().unwrap();};},
               Err(_error) =>  {hprintln!(">").unwrap();
                               }, // skip, but might be good to have actual error
            };
          
        }
    }

    #[task(shared = [ buffer], local = [tx_con])]
    async fn write_con(cx: write_con::Context) {
        let mut buffer = cx.shared.buffer;
        let tx_con = cx.local.tx_con;

        //let _ = buffer.lock(|buf| {tx_con.write_all(&buf).unwrap()});
        let _ = buffer.lock(|buf| {tx_con.write(&[buf[0]]).unwrap()});
        let _ = tx_con.flush().unwrap(); 
        ()
    }
}
