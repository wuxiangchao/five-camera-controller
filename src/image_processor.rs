#![allow(unused)]

use crate::{
    error::{AppError, ImageProcessorError},
    fcu_link::DronePose,
    exif::{build_exif_segment, insert_app1_into_jpeg}
};

use alloc::vec::Vec;

/// Embeds GPS and attitude metadata into a raw JPEG byte buffer.
///
/// This is a complex operation that modifies the JPEG file structure in-place.
///
/// # Arguments
///
/// * `image_data`: A mutable byte slice containing the JPEG image.
/// * `pose`: The drone pose data to embed into the image's EXIF tags.
///
/// # Returns
///
/// `Ok(())` if the metadata was successfully embedded, otherwise an `AppError`.
pub fn embed_gps_metadata(
    image_data: &mut [u8],
    pose: &DronePose,
) -> Result<(), AppError> {
    let exif = build_exif_segment(
        pose.lat,
        pose.lon,
        pose.alt as f64,
        pose.roll as f64,
        pose.pitch as f64,
        pose.yaw as f64);

    defmt::info!("Embedding GPS metadata: {:?}", pose);
    insert_app1_into_jpeg(image_data, &exif)
        .map_err(|_| ImageProcessorError::MetadataEmbedFailed)?;
    
    Ok(())
}