//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    gpio::{Pin, PushPullOutput, bank0::Gpio25},
};
use cortex_m::delay::Delay;

unsafe fn relocate_ram_code() {
    extern "C" {
        static __ram_code_dest_start: u32;
        static __ram_code_dest_end: u32;
        static __ram_code_src_start: u32;
    }
    
    let ptr_dest_start = &__ram_code_dest_start as *const u32;
    let ptr_dest_end = &__ram_code_dest_end as *const u32;
    let ptr_src_start = &__ram_code_src_start as *const u32;
    
    let length = (ptr_dest_end as u32) - (ptr_dest_start as u32);
    
    bsp::hal::rom_data::memcpy44(ptr_dest_start as *mut u32, ptr_src_start, length);
}

#[entry]
fn main() -> ! {
    unsafe { relocate_ram_code(); }
    
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let led_pin = pins.led.into_push_pull_output();

    foobar(delay, led_pin);
}

#[link_section = ".ram_code"]
fn foobar(mut delay: Delay, mut led_pin: Pin<Gpio25, PushPullOutput>) -> ! {
    info!("Executing from {:#010X}", foobar as *const u32 as u32);
    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
