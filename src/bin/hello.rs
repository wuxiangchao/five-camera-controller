#![no_main]
#![no_std]

use five_camera_controller as _;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    five_camera_controller::exit()
}
