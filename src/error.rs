// src/error.rs
use thiserror_no_std::Error;


/// The main error enum for the application, consolidating errors from all modules.
#[derive(Error, Debug, defmt::Format)]
pub enum AppError {
    #[error("Storage Error")]
    Storage(#[from] StorageError),

    #[error("FCU Link Error")]
    FcuLink(#[from] FcuError),

    #[error("Image Processor Error")]
    ImageProcessor(#[from] ImageProcessorError),
}

/// Errors related to SD card storage and filesystem operations.
#[derive(Error, Debug, defmt::Format)]
pub enum StorageError {
    #[error("SD Card initialization failed")]
    SdCardInitFailed,
    #[error("Could not access FAT volume")]
    VolumeAccessFailed,
    #[error("Failed to create directory")]
    DirectoryCreateFailed,
    #[error("Failed to open file")]
    FileOpenFailed,
    #[error("Failed to write to file")]
    FileWriteFailed,
    #[error("Failed to close file")]
    FileCloseFailed,
    #[error("Invalid Camera ID")]
    InvalidCameraId,
}

/// Errors related to communication with the Flight Control Unit (FCU).
#[derive(Error, Debug, defmt::Format)]
pub enum FcuError {
    #[error("UART write error")]
    UartWriteFailed,
    #[error("Failed to parse incoming message")]
    ParseError,
}

/// Errors related to image processing tasks, like embedding metadata.
#[derive(Error, Debug, defmt::Format)]
pub enum ImageProcessorError {
    #[error("Invalid JPEG format")]
    InvalidJpeg,
    #[error("Failed to embed metadata")]
    MetadataEmbedFailed,
}