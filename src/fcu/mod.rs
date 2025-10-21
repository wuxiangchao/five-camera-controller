#![allow(unused)]

use core::fmt;
use crate::{bsp, error::AppError};
use defmt;
use embedded_hal;
use embedded_hal_nb::serial::Read as _;
use embedded_hal_nb::serial::Write as _;
use nb::block;
use stm32h7xx_hal::serial::{Error as HalSerialError, Rx, Tx};
use stm32h7xx_hal::prelude::{_embedded_hal_serial_Read, _embedded_hal_serial_Write};
use embedded_io::{Error as EmbeddedIoErrorTrait, ErrorKind as EmbeddedIoErrorKind, Read as EmbeddedIoRead, ErrorType};
use mavio::{
    dialects::ardupilotmega as mavlink,
    dialects::ardupilotmega::messages::{Heartbeat, GlobalPositionInt, CameraFeedback, SysStatus},
    io::EmbeddedIoReader,
    Frame, MavFrame,
    protocol::Versionless,
    Receiver};
use mavio::error::{Error as MavioError, IoError};
use crate::bsp::pac::UART4;

/// Holds the position and attitude data received from the drone.
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

#[derive(Debug)]
pub enum HalIoError {
    SerialHalError,
}

impl fmt::Display for HalIoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HalIoError::SerialHalError => write!(f, "HAL serial error"),
        }
    }
}

// impl the embedded-io Error trait for our error type
impl EmbeddedIoErrorTrait for HalIoError {
    fn kind(&self) -> EmbeddedIoErrorKind {
        EmbeddedIoErrorKind::Other
    }
}

impl<R> ErrorType for HalEmbeddedReader<R> {
    type Error = HalIoError;
}

// Wrapper that adapts embedded-hal-nb `serial::Read<u8>` to `embedded_io::Read`
pub struct HalEmbeddedReader<R> {
    inner: R,
}

impl<R> HalEmbeddedReader<R> {
    pub fn new(inner: R) -> Self {
        Self { inner }
    }
}

impl EmbeddedIoRead for HalEmbeddedReader<Rx<bsp::pac::UART4>> {
    /// Blocking read into `buf`. We block until at least one byte is available,
    /// then return number of bytes read (here we read at most 1 byte per call).
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        match block!((&mut self.inner).read()) {
            Ok(b) => {
                buf[0] = b;
                Ok(1)
            }
            Err(_e) => Err(HalIoError::SerialHalError),
        }
    }
}


pub struct FcuLink {
    receiver: Receiver<IoError, EmbeddedIoReader<HalEmbeddedReader<Rx<bsp::pac::UART4>>>, Versionless>,
    tx: Tx<bsp::pac::UART4>,
    latest_pose: Option<DronePose>,
}

impl FcuLink {
    /// Creates a new FcuLink manager.
    pub fn new(rx: Rx<bsp::pac::UART4>, tx: Tx<bsp::pac::UART4>) -> Self {
        let hal_reader = HalEmbeddedReader::new(rx);

        let embedded_reader = EmbeddedIoReader::new(hal_reader);

        let receiver = Receiver::new(embedded_reader);

        Self {
            receiver,
            tx,
            latest_pose: None,
        }
    }

    /// Returns the last known pose of the drone.
    pub fn get_latest_pose(&self) -> Option<DronePose> {
        self.latest_pose
    }

    /// Sends a status or error code back to the flight controller.
    pub fn send_error_code(&mut self, code: u8) -> Result<(), AppError> {
        block!(self.tx.write(code)).map_err(|_| AppError::from(crate::error::FcuError::UartWriteFailed))
    }

    /// This function should be called repeatedly from a task or interrupt
    /// to process incoming data from the UART.
    /// It will attempt to read and process one frame per call.
    pub fn process_incoming_data(&mut self) {
        match self.receiver.recv() {
            Ok(frame) => {
                self.handle_mavlink_frame(&frame);
            }
            Err(e) => {
                match e {
                    MavioError::Frame { .. } => {
                        defmt::warn!("MAVLink Frame error");
                    }
                    MavioError::Io(_) => {

                    }
                    _ => {
                        defmt::warn!("MAVLink receive error");
                    }
                }
            }
        }
    }

    /// Decodes a MAVLink frame and updates the internal state.
    fn handle_mavlink_frame(&mut self, frame: &Frame<Versionless>) {
        match frame.message_id() {
            Heartbeat::ID => {
                if let Ok(msg) = frame.decode_message::<Heartbeat>(){
                    defmt::info!("Receiving heartbeat");
                }
            }
            GlobalPositionInt::ID =>{
                if let Ok(msg) = frame.decode_message::<GlobalPositionInt>(){
                    self.latest_pose = Some(DronePose {
                        lat: msg.lat as f64 / 1e7,
                        lon: msg.lon as f64 / 1e7,
                        alt: msg.alt as f32 / 1000.0,
                        relative_alt: msg.relative_alt as f32 / 1000.0,
                        ..Default::default()
                    });
                    defmt::info!("Updated DronePose: {:?}", self.latest_pose);
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
            _ => {  }
        }
    }
}