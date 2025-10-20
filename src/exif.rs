extern crate alloc_no_stdlib;
use alloc::vec::Vec;
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

    // 示例写入简单的 ASCII 信息（实际可替换为标准 TIFF IFD）
    let info = format_exif_ascii(lat, lon, alt, roll, pitch, yaw);
    exif.extend_from_slice(info.as_bytes());

    // 回填长度
    let len = (exif.len() - 2) as u16;
    exif[2] = (len >> 8) as u8;
    exif[3] = (len & 0xFF) as u8;

    exif
}

/// 将浮点数据转为简化的 ASCII EXIF 内容（自定义标签）
fn format_exif_ascii(lat: f64, lon: f64, alt: f64, roll: f64, pitch: f64, yaw: f64) -> alloc::string::String {
    use alloc::string::String;
    let mut s = String::new();
    s.push_str("GPSLatitude=");
    s.push_str(&float_to_str(lat));
    s.push_str("; GPSLongitude=");
    s.push_str(&float_to_str(lon));
    s.push_str("; GPSAltitude=");
    s.push_str(&float_to_str(alt));
    s.push_str("; Roll=");
    s.push_str(&float_to_str(roll));
    s.push_str("; Pitch=");
    s.push_str(&float_to_str(pitch));
    s.push_str("; Yaw=");
    s.push_str(&float_to_str(yaw));
    s
}

/// 简易浮点转字符串（no_std-friendly）
fn float_to_str(v: f64) -> alloc::string::String {
    use alloc::string::String;
    let mut s = String::new();
    let mut int_part = v as i32;
    let mut frac_part = fabs(v - (int_part as f64)) * 10000.0;
    if v < 0.0 { s.push('-'); }
    s.push_str(itoa(int_part.abs()));
    s.push('.');
    s.push_str(itoa(frac_part as i32));
    s
}

/// 整数转字符串（最小依赖）
fn itoa(mut n: i32) -> &'static str {
    use core::fmt::Write;
    static mut BUF: [u8; 12] = [0; 12];
    let mut idx = 11;
    unsafe {
        if n == 0 {
            BUF[idx] = b'0';
            return core::str::from_utf8_unchecked(&BUF[idx..]);
        }
        while n > 0 {
            BUF[idx] = b'0' + (n % 10) as u8;
            n /= 10;
            idx -= 1;
        }
        core::str::from_utf8_unchecked(&BUF[idx + 1..])
    }
}

/// 将 EXIF APP1 段插入 JPEG 数据（Vec 版本）
pub fn insert_app1_into_jpeg(jpeg_data: &mut [u8], app1: &[u8]) -> Result<(), &'static str> {
    if jpeg_data.len() < 2 || jpeg_data[0] != 0xFF || jpeg_data[1] != 0xD8 {
        return Err("Invalid JPEG header");
    }

    // 查找插入点（SOI 后）
    let insert_pos = 2;
    jpeg_data.splice(insert_pos..insert_pos, app1.iter().cloned());
    Ok(())
}
