#![allow(unused)]

extern crate alloc_no_stdlib;
use alloc::vec::Vec;
use alloc::string::{String, ToString};
use libm::fabs;

/// 构造一个简单的 EXIF APP1 段（含经纬度 + 姿态角）
pub fn build_exif_segment(lat: f64, lon: f64, alt: f64, roll: f64, pitch: f64, yaw: f64) -> Vec<u8> {
    let mut exif = Vec::new();

    // APP1 marker
    exif.extend_from_slice(&[0xFF, 0xE1]);

    // 预留长度（稍后回填）
    exif.extend_from_slice(&[0x00, 0x00]);

    // EXIF header
    exif.extend_from_slice(b"Exif\0\0");

    // 示例写入简单的 ASCII 信息
    let info = format_exif_ascii(lat, lon, alt, roll, pitch, yaw);
    exif.extend_from_slice(info.as_bytes());

    // 回填长度
    let len = (exif.len() - 2) as u16;
    exif[2] = (len >> 8) as u8;
    exif[3] = (len & 0xFF) as u8;

    exif
}

/// [已修复] 将浮点数据转为简化的 ASCII EXIF 内容（自定义标签）
fn format_exif_ascii(lat: f64, lon: f64, alt: f64, roll: f64, pitch: f64, yaw: f64) -> String {
    use alloc::format;

    let mut lat_buf = ryu::Buffer::new();
    let mut lon_buf = ryu::Buffer::new();
    let mut alt_buf = ryu::Buffer::new();
    let mut roll_buf = ryu::Buffer::new();
    let mut pitch_buf = ryu::Buffer::new();
    let mut yaw_buf = ryu::Buffer::new();

    format!(
        "GPSLatitude={}; GPSLongitude={}; GPSAltitude={}; Roll={}; Pitch={}; Yaw={}",
        lat_buf.format(lat),
        lon_buf.format(lon),
        alt_buf.format(alt),
        roll_buf.format(roll),
        pitch_buf.format(pitch),
        yaw_buf.format(yaw)
    )
}


/// 将EXIF APP1段插入JPEG 数据（Vec 版本）
pub fn insert_app1_into_jpeg(
    jpeg_data: &mut Vec<u8>,
    app1: &[u8]) -> Result<(), &'static str> {

    if jpeg_data.len() < 2 || jpeg_data[0] != 0xFF || jpeg_data[1] != 0xD8 {
        return Err("Invalid JPEG header");
    }

    let insert_pos = 2;

    jpeg_data.splice(insert_pos..insert_pos, app1.iter().cloned());
    Ok(())
}