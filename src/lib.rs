#![no_main]
#![no_std]

#![allow(static_mut_refs)]
extern crate alloc;

use embedded_alloc::Heap;

use defmt_rtt as _; // global logger
use stm32h7xx_hal as _;
use panic_probe as _;

// all pub mod
pub mod bsp;
pub mod camera;
pub mod config;
pub mod error;
pub mod fcu_link;
pub mod image_processor;
pub mod storage;

pub mod exif;

// 全局堆分配器
#[global_allocator]
static HEAP: Heap = Heap::empty();

// 初始化堆的函数（供 RTIC 的 init() 调用）
pub fn init_heap() {
    extern "C"{
        static mut __heap_start: u8;
        static mut __heap_end: u8;
    }

    unsafe {
        let heap_start = &mut __heap_start as *mut u8 as usize;
        let heap_size = (&__heap_end as *const u8 as usize) - heap_start;
        HEAP.init(heap_start, heap_size);
    }
}

/// Terminates the application and makes a semihosting-capable debug tool exit
/// with status code 0.
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
