//!  Transmit a simple message with LoRa using crate radio_sx127x (on SPI).
//!
//!  Status: tested working on stm32g474 and stm32f411 in debug with probe, Sept 5, 2024.
//!          Does not work on battery because hprintln requires debug probe.
//!     lora.delay_ms(5) CONVERTS 5 TO SECONDS ON  STM32F411 BUT MILLISECONDS ON  STM32G474
//!
//!  The setup() functions make the application code common. They are in src/setup_all*.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the setup() corresponding to the HAL for details on pin connections.
//!  The setup is using  sck, miso, mosi, cs, reset and D00, D01. Not yet using  D02, D03
//!  
//!  The same setup function is used for examples lora_spi_send, lora_spi_receive, and
//!  lora_spi_gps (if the HAL setting is the same). The following is for all examples.
//!
//!  See FREQUENCY in src/lora.rs to set the channel.
//!  Before running, check  FREQUENCY to be sure you have a channel setting appropriate for
//!  your country, hardware and any testing sender/receiver on the other end of the communication.
//!
//!  To build, link, loaded and run see instructions in  README.md, briefly:
//!      - replace xxx with the example name  lora_spi_send, lora_spi_receive, or lora_spi_gps.
//!      - set TARGET, HAL, and MCU from a line in the table in README.md.
//!      - no-default-features is because some default-features require std.
//!
//!    cargo build --no-default-features --target $TARGET --features=$HAL,$MCU,compat --example xxx [ --release ]
//!
//!  in another window
//!    openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg
//!
//!    cargo  run  --no-default-features --target $TARGET --features $HAL,$MCU,compat --example xxx [ --release ]
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

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_reset;

use cortex_m_rt::entry;

// semihosting for debugging  requires connection to computer and openocd. 
// Comment out next and hprintln statements to run on battery.
//use cortex_m_semihosting::*;

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


//   //////////////////////////////////////////////////////////////////////

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED};

use rust_integration_testing_of_examples::lora::{CONFIG_PA, CONFIG_RADIO, CONFIG_LORA, CONFIG_CH, FREQUENCY, MODE};
//use rust_integration_testing_of_examples::lora::{CONFIG_RADIO};

//   //////////////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {
    let dp =Peripherals::take().unwrap();
    let (mut led, spi, spiext, mut delay) = setup::led_spi_spiext_delay_from_dp(dp); 
    led.off();

    //hprintln!("start delay.delay_ms(5)").unwrap();
    delay.delay_ms(5);
    //hprintln!(" end  delay.delay_ms(5)").unwrap();

    // cs should be called nss
    let lora = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, 
                       &CONFIG_RADIO ); 

    let mut lora =  match lora {
            Ok(lr)  => { //hprintln!("lora setup completed.").unwrap();
                         lr
                       } 
            Err(e) =>  { //hprintln!("Error in lora setup. {:?}", e).unwrap();
                         panic!("{:?}", e)
                       }
    };
 
    //let mut lora = lora.unwrap();
 
    //delay is available in lora

    
   
    // print out configuration (for debugging)
//    hprintln!("frequency          {:?}", lora.get_frequency());

 //   use radio_sx127x::device::regs::Register;
 //
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

    // transmit something

    //let buffer = &[0xaa, 0xbb, 0xcc];

    let message = b"Hello, LoRa!";

    //let mut buffer = [0;100];      //Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    //for (i,c) in message.chars().enumerate() {
    //	buffer[i] = c as u8;
    //	}

    loop {
        match lora.start_transmit(message) {
            Ok(_b)   => { //hprintln!("start_transmit").unwrap()
                        } 
            Err(_e)  => { //hprintln!("Error in lora.start_transmit()").unwrap()
                        }
        };
        //hprintln!("start_transmit done").unwrap();

        lora.delay_ms(1); // without some delay next returns bad. (interrupt may also be an option)

        match lora.check_transmit() {
            Ok(b)   => {if b {//hprintln!("TX good").unwrap()
                             } 
                        else {//hprintln!("TX bad").unwrap()
                             }
                       }
            Err(_e) => {
                        //hprintln!("Error in lora.check_transmit(). Should return True or False.").unwrap()
                       }
        };
        //hprintln!("check_transmit done").unwrap();

        //lora.delay_ms(5000);
        lora.delay_ms(5);
        //hprintln!("re-loop").unwrap();
    }
}
