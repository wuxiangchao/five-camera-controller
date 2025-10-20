#![allow(unused)]

use alloc::vec;
use crate::{bsp, config};
use alloc::vec::Vec;
use stm32h7xx_hal::gpio::{Output, PushPull};
use crate::error::AppError;

pub struct CameraController {
    triggers: bsp::CameraTriggers,
}

impl CameraController {
    /// Creates a new CameraController.
    pub fn new(triggers: bsp::CameraTriggers) -> Self {
        Self { triggers }
    }

    /// Triggers all five cameras simultaneously by sending a short pulse.
    pub fn trigger_all(&mut self) {
        self.triggers.cam1.set_high();
        self.triggers.cam2.set_high();
        self.triggers.cam3.set_high();
        self.triggers.cam4.set_high();
        self.triggers.cam5.set_high();

        cortex_m::asm::delay(config::TRIGGER_PULSE_WIDTH_US);

        self.triggers.cam1.set_low();
        self.triggers.cam2.set_low();
        self.triggers.cam3.set_low();
        self.triggers.cam4.set_low();
        self.triggers.cam5.set_low();
    }

    pub fn read_image_from_camera(cam_id: usize) -> Result<Vec<u8>, AppError> {
        defmt::info!("Reading image from camera {} (STUB)", cam_id);
        // TODO: 在此处实现真实的相机驱动逻辑

        // --- 当前返回虚拟数据以供测试 ---
        let mut image_data_vec: Vec<u8> = vec![0xFF, 0xD8, 0x00, 0x00]; // JPEG SOI
        image_data_vec.resize(1024 * 50, cam_id as u8); // 模拟一张50KB的图片
        Ok(image_data_vec)
        // 在真实场景中，如果读取失败，应返回类似 Err(AppError::ImageReadFailed) 的错误
    }
}