// Following https://github.com/stm32-rs/stm32f4xx-hal/blob/master/examples/rtic-serial-dma-rx-idle.rs
//     BUT will eventually attempt tx as well as rx
//
// This example implement simple and safe method receiving data with unknown length by UART.
// The data received by using DMA, and IDLE event denote end of data packet.
// See https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx for details.

// If you use big buffers, it is recommended to add memory pools (allocators) and use
// lock-free queues to send buffer without memcpy.

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//  peripherals = true,  is the default, but to reduce code size set false if peripherals are not needed.

use rtic::app;
use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000); 

//  TIM3 is not yet used
#[cfg_attr(feature = "stm32f0xx", rtic::app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", rtic::app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", rtic::app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", rtic::app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", rtic::app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", rtic::app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", rtic::app(device = stm32g4xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", rtic::app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", rtic::app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", rtic::app(device = stm32l1xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", rtic::app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;
 
    #[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
    use stm32f4xx_hal::{
        dma::{config::DmaConfig, PeripheralToMemory, Stream2, StreamsTuple, Transfer},
        pac::{Peripherals, DMA2, USART1},
        prelude::*,
        rcc::RccExt,
        serial,
    };

    #[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
    const MONOTICK: u32 = 1000;       // 1000 Hz / 1 ms granularity


    const BUFFER_SIZE: usize = 100;

    type RxTransfer = Transfer<
        Stream2<DMA2>,
        4,
        serial::Rx<USART1>,
        PeripheralToMemory,
        &'static mut [u8; BUFFER_SIZE],
    >;

    #[shared]
    struct Shared {
        #[lock_free]
        rx_transfer: RxTransfer,
    }

    #[local]
    struct Local {
        rx_buffer: Option<&'static mut [u8; BUFFER_SIZE]>,
    }


    #[init(local = [
        rx_pool_memory: [u8; 400] = [0; 400],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let core = cx.core;
        let dp: Peripherals = cx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());

        let gpioa = dp.GPIOA.split();

        // Initialize UART with DMA events
        let rx_pin = gpioa.pa10;
        let mut rx = dp
            .USART1
            .rx(
                rx_pin,
                serial::Config::default()
                    .baudrate(9600.bps())
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();

        // Listen UART IDLE event, which will be call USART1 interrupt
        rx.listen_idle();

        let dma2 = StreamsTuple::new(dp.DMA2);

        // Note! It is better to use memory pools, such as heapless::pool::Pool. But it not work with embedded_dma yet.
        // See CHANGELOG of unreleased main branch and issue https://github.com/japaric/heapless/pull/362 for details.
        let rx_buffer1 = cortex_m::singleton!(: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();
        let rx_buffer2 = cortex_m::singleton!(: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();

        // Initialize and start DMA stream
        let mut rx_transfer = Transfer::init_peripheral_to_memory(
            dma2.2,
            rx,
            rx_buffer1,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        rx_transfer.start(|_rx| {});

        (Shared { rx_transfer }, Local { rx_buffer: Some(rx_buffer2) })
    }

    // Important! USART1 and DMA2_STREAM2 should the same interrupt priority!
    #[task(binds = USART1, priority=1, local = [rx_buffer],shared = [rx_transfer])]
    fn usart1(mut cx: usart1::Context) {
        let transfer = &mut cx.shared.rx_transfer;

        if transfer.is_idle() {
            // Calc received bytes count
            let bytes_count = BUFFER_SIZE - transfer.number_of_transfers() as usize;

            // Allocate new buffer
            let new_buffer = cx.local.rx_buffer.take().unwrap();

            // Replace buffer and restart DMA stream
            let (buffer, _) = transfer.next_transfer(new_buffer).unwrap();

            // Get slice for received bytes
            let _bytes = &buffer[..bytes_count];

            // Do something with received bytes
            // For example, parse it or send (buffer, bytes_count) to lock-free queue.

            // Free buffer
            *cx.local.rx_buffer = Some(buffer);
        }
    }

    #[task(binds = DMA2_STREAM2, priority=1,shared = [rx_transfer])]
    fn dma2_stream2(mut cx: dma2_stream2::Context) {
        let transfer = &mut cx.shared.rx_transfer;

        if transfer.is_fifo_error() {
            transfer.clear_fifo_error();
        }
        if transfer.is_transfer_complete() {
            transfer.clear_transfer_complete();

            // Buffer is full, but no IDLE received!
            // You can process this data or discard data (ignore transfer complete interrupt and wait IDLE).

            // Note! If you want process this data, it is recommended to use double buffering.
            // See Transfer::init_peripheral_to_memory for details.
        }
    }
}
