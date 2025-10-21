#![allow(unused)]

use core::fmt;
use cortex_m::prelude::_embedded_hal_serial_Write;
use crate::{bsp, error::AppError};
use defmt;
use nb::block; // 用于 send_error_code
use stm32h7xx_hal::serial::Tx;
use embedded_io::{Error as EmbeddedIoErrorTrait, ErrorKind as EmbeddedIoErrorKind, Read as EmbeddedIoRead, ErrorType};
use mavio::{
    dialects::ardupilotmega as mavlink,
    dialects::ardupilotmega::messages::{Heartbeat, GlobalPositionInt, CameraFeedback, SysStatus},
    io::EmbeddedIoReader,
    Frame, MavFrame,
    protocol::Versionless,
    Receiver,
    error::Error as MavioError, // 使用此别名
    error::IoError, // 这是 Receiver 的错误类型
};
use crate::bsp::pac::UART4;
use heapless::{Deque, spsc};
use mavio::prelude::MaybeVersioned;
// 用于 DequeReader

/// 持有从无人机接收到的位置和姿态数据。
#[derive(Copy, Clone, Debug, defmt::Format, Default)]
pub struct DronePose {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32, // Altitude (AMSL)
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub relative_alt: f32, // Altitude above home
}

/// DequeReader 的自定义 IO 错误类型
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum DequeIoError {
    /// 一个通用错误（尽管 DequeReader::read 目前不会失败）
    ReadError,
}

impl fmt::Display for DequeIoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DequeIoError::ReadError => write!(f, "DequeReader read error"),
        }
    }
}

impl EmbeddedIoErrorTrait for DequeIoError {
    fn kind(&self) -> EmbeddedIoErrorKind {
        EmbeddedIoErrorKind::Other
    }
}


pub struct DequeReader<'a, const N: usize> {
    consumer: &'a mut spsc::Consumer<'a, u8, N>,
}

impl<'a, const N: usize> DequeReader<'a, N> {
    pub fn new(consumer: &'a mut spsc::Consumer<'a, u8, N>) -> Self {
        Self { consumer }
    }
}

impl<'a, const N: usize> ErrorType for DequeReader<'a, N> {
    type Error = DequeIoError;
}

impl<'a, const N: usize> EmbeddedIoRead for DequeReader<'a, N> {
    /// 阻塞式读取 `buf`。我们阻塞直到至少一个字节可用。
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        // 循环（阻塞），直到可以从队列中取出一个字节。
        let byte = loop {
            if let Some(b) = self.consumer.dequeue() {
                break b;
            }
            cortex_m::asm::nop();
        };

        // 我们读取了一个字节
        buf[0] = byte;
        Ok(1)
    }
}


pub struct FcuLink {
    tx: Tx<bsp::pac::UART4>,
}

impl FcuLink {
    /// 创建一个新的 FcuLink 辅助程序。
    pub fn new(tx: Tx<bsp::pac::UART4>) -> Self {
        Self { tx }
    }

    /// 向飞控发送状态或错误代码。
    /// 这仍然是阻塞的，但假定它从软件任务调用，而不是 ISR。
    pub fn send_error_code(&mut self, code: u8) -> Result<(), AppError> {
        block!(self.tx.write(code)).map_err(|_| AppError::from(crate::error::FcuError::UartWriteFailed))
    }

    /// 此函数由 main.rs 中的 'process_mavlink' 任务*调用*。
    /// 它执行接收 MAVLink 帧的阻塞工作。
    pub fn process_incoming_data(
        &mut self,
        receiver: &mut Receiver<IoError, EmbeddedIoReader<DequeReader<1024>>, Versionless>, // 接收 Receiver
        shared_pose: &mut Option<DronePose>
    ) {
        // 这个调用现在是安全的。它阻塞在 DequeReader 上，
        // DequeReader阻塞在Deque上，
        match receiver.recv() {
            Ok(frame) => {
                self.handle_mavlink_frame(&frame, shared_pose);
            }
            Err(e) => {
                match e {
                    MavioError::Io(_) => {
                        defmt::warn!("MAVLink IO error (from DequeReader)");
                    }
                    MavioError::Frame(frame_error) => {
                        defmt::warn!("MAVLink Frame error: {:?}", frame_error);
                    }
                    _ => {
                        defmt::warn!("MAVLink receive error");
                    }
                }
            }
        }
    }

    /// 解码 MAVLink 帧并更新内部状态。
    fn handle_mavlink_frame(
        &mut self,
        frame: &Frame<Versionless>,
        shared_pose: &mut Option<DronePose>
    ) {
        match frame.message_id() {
            Heartbeat::ID => {
                if let Ok(msg) = frame.decode_message::<Heartbeat>(){
                    defmt::trace!("Receiving heartbeat");
                }
            }
            GlobalPositionInt::ID =>{
                if let Ok(msg) = frame.decode_message::<GlobalPositionInt>(){
                    *shared_pose = Some(DronePose {
                        lat: msg.lat as f64 / 1e7,
                        lon: msg.lon as f64 / 1e7,
                        alt: msg.alt as f32 / 1000.0,
                        relative_alt: msg.relative_alt as f32 / 1000.0,
                        ..Default::default() // roll, pitch, yaw 保持 0.0
                    });
                    defmt::info!("Updated DronePose: {:?}", *shared_pose);
                }
            }
            CameraFeedback::ID => {
                if let Ok(msg) = frame.decode_message::<CameraFeedback>(){
                    defmt::info!("Camera feedback");
                }
            }
            SysStatus::ID => {
                if let Ok(msg) = frame.decode_message::<SysStatus>(){
                    defmt::info!("System status");
                }
            }
            _ => {

            }
        }
    }
}