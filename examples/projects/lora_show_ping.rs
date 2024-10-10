//!  Transmit a simple message with LoRa using crate radio_sx127x (on SPI).
//!  Display on ssd if it is detected on I2C2.
//!  Status: WIP, adapted from lora_spi_send. Intend to incorporate into sensor projects.
//!
//!  The setup is using  sck, miso, mosi, cs, reset and D00, D01. Not yet using  D02, D03
//!  The radio seems to need a pull-up resistor on the cs pin so it does not float on startup.
//!  The symptom is  InvalidDevice(0) or InvalidDevice(252) returned by Sx127x::spi(spi,...).
//!  
//!  See FREQUENCY in src/lora.rs to set the channel.
//!  Before running, check  FREQUENCY to be sure you have a channel setting appropriate for
//!  your country, hardware and any testing sender/receiver on the other end of the communication.
//!
//!    cargo build --no-default-features --target $TARGET --features=$HAL,$MCU --example lora_show_ping [ --release ]
//!
//!  in another window
//!    openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg
//!
//!    cargo  run  --no-default-features --target $TARGET --features $HAL,$MCU --example lora_show_ping [ --release ]
//!
//!  If --release is omitted then some MCUs do not have sufficient memory and loading results in
//!       '.rodata will not fit in region FLASH '
//!  Even with sufficient memory the code without --release is slower and may result in errors.

//https://www.rfwireless-world.com/Tutorials/LoRa-channels-list.html
// channels are as follows
//   'CH_00_900': 903.08, 'CH_01_900': 905.24, 'CH_02_900': 907.40,
//   'CH_03_900': 909.56, 'CH_04_900': 911.72, 'CH_05_900': 913.88,
//   'CH_06_900': 916.04, 'CH_07_900': 918.20, 'CH_08_900': 920.36,
//   'CH_09_900': 922.52, 'CH_10_900': 924.68, 'CH_11_900': 926.84, 'CH_12_900': 915,
//
//   'CH_10_868': 865.20, 'CH_11_868': 865.50, 'CH_12_868': 865.80,
//   'CH_13_868': 866.10, 'CH_14_868': 866.40, 'CH_15_868': 866.70,
//   'CH_16_868': 867   , 'CH_17_868': 868   ,

// Errata https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R000000HSPv/sqi9xX0gs6hgzl2LoPwCK0TS9GDPlMwsXmcNzJCMHjw
// Semtec  stores value 0x12 at address 0x42 to indicate version V1b. 
// V1a was pre-production engineering samples

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_reset;

use cortex_m_rt::entry;
use cortex_m_semihosting::*;
use cortex_m::asm; // for delay

//use cortex_m_semihosting::{debug, hprintln};
//use cortex_m_semihosting::{hprintln};

use embedded_hal::delay::DelayNs;

//use radio::Transmit;  // trait needs to be in scope to find  methods start_transmit and check_transmit.
use radio_sx127x::Transmit;  // trait needs to be in scope to find  methods start_transmit and check_transmit.

use radio_sx127x::{
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
};

// for config examination on debugging
use radio_sx127x::{
    device::regs::Register,
   // read_register, get_mode, get_signal_bandwidth, get_coding_rate_4, get_spreading_factor,
    
};


//    /////////////////////  ssd

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html re fonts.
    
    type  DisplaySize = ssd1306::prelude::DisplaySize128x64;

    //common display sizes are 128x64 and 128x32
    const DISPLAYSIZE: DisplaySize = DisplaySize128x64;

    use embedded_graphics::{
        //mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_9X15 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};


//   //////////////////////////////////////////////////////////////////////

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, MONOCLOCK, I2c2Type};

use rust_integration_testing_of_examples::lora::{CONFIG_PA, CONFIG_RADIO, CONFIG_LORA, CONFIG_CH, FREQUENCY, MODE};
//use rust_integration_testing_of_examples::lora::{CONFIG_RADIO};


//    /////////////////////   constants and types

    const READ_INTERVAL: u32 = 5;  // used as seconds

    const BLINK_DURATION: u16 = 20;  // used as milliseconds

type  DisplayType = Ssd1306<I2CInterface<I2c2Type>, DisplaySize, BufferedGraphicsMode<DisplaySize>>;

   // type DisplayType = Ssd1306<impl WriteOnlyDataCommand, ssd1306::size::DisplaySize, BufferedGraphicsMode<ssd1306::size::DisplaySize>>;

//   //////////////////////////////////////////////////////////////////////

    fn show_message(
        text: &str,   //text_style: MonoTextStyle<BinaryColor>,
        //disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        disp: &mut Option<DisplayType>,
    ) -> ()
 //    where
 //        S: ssd1306::size::DisplaySize,  //trait
    {
       if disp.is_some() { // show_message does nothing if disp is None. 
           //let mut disp = *disp;   
           // workaround. build here because text_style cannot be shared
           let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
        
           disp.as_mut().expect("REASON").clear_buffer();
           Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
                   .draw(disp.as_mut().expect("REASON"))
                   .unwrap();

           disp.as_mut().expect("REASON").flush().unwrap();
       };
       ()
    }

//   //////////////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {
    let dp =Peripherals::take().unwrap();
    let (_i2c1, i2c2, mut led, spi, spiext, mut delay) = setup::i2c1_i2c2_led_spi_spiext_delay_from_dp(dp);
    led.off();
    delay.delay_ms(2000); // treated as ms
    led.blink(BLINK_DURATION, &mut delay);

    /////////////////////   ssd

    let interface = I2CDisplayInterface::new(i2c2);

    let mut z = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0);

    //z.init().unwrap();

    //hprintln!("match z.init()  ...").unwrap();

// stalls here if there is no display but works when display is present
//    let mut disp: Option<DisplayType> = match z.init() {
//            Ok(_d)  => {Some(z.into_buffered_graphics_mode())} 
//            Err(_e) => {None}
//    };

    let mut zz = z.init();
   // hprintln!("z.init() done  ...").unwrap();
    let mut disp: Option<DisplayType> = match zz {
            Ok(d)  => {Some(z.into_buffered_graphics_mode())} 
            Err(_e) => {None}
    };

    //hprintln!("disp set  ...").unwrap();
 
    show_message("lora_show_ping", &mut disp);

    delay.delay_ms(2000); // treated as ms


    /////////////////////   lora

    // cs is named nss on many radio_sx127x module boards
    let z = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, 
                       &CONFIG_RADIO ); 

    let mut lora =  match z {
            Ok(lr)  => {show_message("lora setup ok", &mut disp);
                        //hprintln!("lora setup completed.").unwrap();
                        lr
                       } 
            Err(e)  => {show_message("lora setup Error", &mut disp);
                        //hprintln!("Error in lora setup. {:?}", e).unwrap();
                        asm::bkpt();
                        panic!("{:?}", e) 
                       }
    };
 

    //delay is now available in lora BUT treats arg as seconds not ms!!
    lora.delay_ms(1);  // this is being treated as seconds
    
   
    // print out configuration (for debugging)
//    hprintln!("frequency          {:?}", lora.get_frequency());

//    use radio_sx127x::device::regs::Register;

//    let v = lora.lora_get_config();
//    hprintln!("configuration {:?}", v).unwrap();
// 
//    hprintln!("channel      {}", lora.get_channel()).unwrap();
// 
//    hprintln!("mode             {}",    lora.get_mode()).unwrap();
//    hprintln!("mode             {}",    lora.read_register(Register::RegOpMode.addr())).unwrap();
//    hprintln!("bandwidth        {:?}",  lora.get_signal_bandwidth()).unwrap();
//    hprintln!("coding_rate      {:?}",  lora.get_coding_rate_4()).unwrap();
//    hprintln!("spreading_factor {:?}",  lora.get_spreading_factor()).unwrap();
//    hprintln!("invert_iq        {:?}",  lora.get_invert_iq()).unwrap();
//    hprintln!("tx_power         {:?}",  lora.get_tx_power()).unwrap();


    /////////////////////   transmit

    let message = b"Hello, LoRa!";

    //let mut buffer = [0;100];      //Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    //for (i,c) in message.chars().enumerate() {
    //	buffer[i] = c as u8;
    //	}

    loop {

        //hprintln!("start_transmit ...").unwrap();
        match lora.start_transmit(message) {
            Ok(_b)   => {//show_message("start_transmit ok", &mut disp);
                         //hprintln!("lora.start ok").unwrap()
                        } 
            Err(_e)  => {show_message("start_transmit error", &mut disp);
                         //hprintln!("Error in lora.start_transmit()").unwrap()
                        }
        };
        //hprintln!("... done").unwrap();

        lora.delay_ms(10); // treated as seconds. Without some delay next returns bad. (interrupt may also be an option)

        match lora.check_transmit() {
            Ok(b)   => {if b {show_message("TX good", &mut disp);
                              //hprintln!("TX good").unwrap(); 
                             }
                        else {show_message("TX bad", &mut disp);
                              //hprintln!("TX bad").unwrap()
                             }
                       }
            Err(_e) => {show_message("check_transmit Fail", &mut disp);
                        //hprintln!("check_transmit() Error. Should return True or False.").unwrap()
                       }
        };
        //hprintln!("check_transmit done").unwrap();

        lora.delay_ms(READ_INTERVAL);  // treated as seconds
        //hprintln!("re-loop").unwrap();
    }
}
