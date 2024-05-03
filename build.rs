// This file (build.rs) needs to be in the package root.
// It is called by cargo before the linker and is used to arrange for the linker to 
// find the proper memory.x file. The memory.x files are assumed to be in directories
// memoryMaps/xxx/ in the package root, where xxx is replaced by one of the MCU directories
// indicated in the main function below. (For example,  memoryMaps/STM32F401/memory.x )
//
// Alternately, the xxx directory can be specified by setting an environment variable $MEMMAP.
// Note that these directory names should be in upper case because CARGO_FEATURE_* env variables 
// are in upper case. For consistency, the $MEMMAP environment variable is also converted to upper case.
//
// The location to the memory.x file is added to the linker's search path rather than copying
// the file to the target directory. This allows for the possibility that there may be different
// memory.x files for a given target triple.  It also helps to prevent memory.x files
// being left in the target directory where they can be mistakenly found by the linker.
//
// Case sensitive matching is used but MEMMAP is first converted to upper case. 
// The code uses the first location found in the order 
//
//        - environment variable $MEMMAP, 
//
//        - the first directory in MemoryMaps matching a CARGO_FEATURE_* env variable
//
//        - otherwise no search path is added and memory.x will be found as usual in the root directory.
//
// This means the  $MEMMAP value can be used as a standard way to override the CARGO_FEATURE_* value.
//
// It also provides a mechanism to specify a location in cases where MCU features are not provided,
// e.g.  to run examples in a device crate:
//    MEMMAP=STM32F103  cargo build --target thumbv7m-none-eabi  --release  --examples
//    MEMMAP=stm32f103  cargo build --target thumbv7m-none-eabi  --release  --example blink
//
// Note however, this mechanism only finds the memory.x for the linker.  It does not override 
// a `use` statement in the code. So if example code specifies use stm32f1xx_hal and MEMMAP=stm32f411 
// there will be a mismatch of the compile target and the linker memory map. (Sometimes works but 
// not recommended.)
//
// Using $MEMMAP allows setting any special hardware specific map. Note that setting $MEMMAP to an non-existent
// location for the memory.x file will result in a linker error  
//       linking with `rust-lld` failed: exit status: 1 ... cannot find linker script memory.x  >>> INCLUDE memory.x
//
// This is also the usual error message when no memory.x file is found.

use std::env;
//DBG use std::io::Write;         //needed for debugging

fn main() {

    let mcus = [
        "STM32F042",
        "STM32F030XC",
        "STM32F100",
        "STM32F101",
        "STM32F103",
        "STM32F303XC",
        "STM32F401",
        "STM32F411",
        "STM32F722",
        "STM32G081",
        "STM32G431XB",
        "STM32G473",
        "STM32G474XE",
        "STM32H742",
        "STM32L0X2",
        "STM32L100",
        "STM32L151",
        "STM32L422",
        "LM3S6965",
        "GD32VF103CB",
        "GD32VF103C8",
        "GD32VF103_EXTRA",
    ];

    let pre = "CARGO_FEATURE_".to_owned();

    // The BUILD.RS.log text file is just to record debugging information
    //DBG let mut df = std::fs::File::create("BUILD.RS.log").unwrap();

    // For debugging. Write all CARGO_FEATURE_  mcu variables to file
    //DBG for m in &mcus {
    //DBG   match env::var_os(pre.clone() + m) {
    //DBG      None    => df.write(format!("{}{} is not set\n", pre, m).as_bytes()).unwrap(),
    //DBG      Some(x) => df.write(format!("{}{} is {:?}\n", pre, m, x).as_bytes()).unwrap()
    //DBG      };
    //DBG   };

    // For debugging. Write all env variables to file
    //DBG   df.write(format!("env::vars() gives\n").as_bytes()).unwrap();
    //DBG   for (key, value) in env::vars() {
    //DBG       df.write(format!("   {:?}: {:?}\n", key, value).as_bytes()).unwrap();
    //DBG       };

    let mut mem_map : String = "".to_string();

    for (key, value) in env::vars() {
        if key.eq("MEMMAP")  {
           //DBG df.write(format!(" MEMMAP is {:?}: {:?}\n", key, value).as_bytes()).unwrap();
           mem_map = value.to_uppercase();
           break;
        };
    };

    // If location has not been set by MEMMAP, compare mcus elements against CARGO_FEATURE_* env 
    // variables to determine directory of memory.x file to use.
   
    if mem_map.is_empty() {
       for m in &mcus {
           let v = env::var_os(pre.clone() + m);
           if v.is_some() {
               mem_map = m.to_string();
               break;
           };
       }
    };

    let indir = if mem_map.is_empty() {"".to_string()}
                else {"memoryMaps/".to_owned() + &mem_map};

    //DBG df.write(format!(" {:?}\n",mem_map).as_bytes()).unwrap();

 
    // Adding an empty search path causes problems compiling the crate, so skip if memory.x not found.
    // This allows 'cargo build  --features $MCU ' to work
    // but do not expect to compile examples. (There will be a 'cannot find linker script memory.x' error.)

    if !indir.is_empty() {
        let infile = indir.clone() + "/memory.x";
        //DBG df.write(format!("using {:?} \n", infile).as_bytes()).unwrap();

        println!("cargo:rustc-link-search={}", indir);
        println!("cargo:rerun-if-changed=build.rs");
        println!("cargo:rerun-if-changed={}", infile);
    } else {
        //DBG  df.write(format!("mcu NOT found condition.\n").as_bytes()).unwrap();
        println!();
    }
}
