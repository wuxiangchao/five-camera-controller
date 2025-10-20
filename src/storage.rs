#![allow(unused)]
use heapless::String;
use core::fmt::Write;
use fatfs::{self, FileSystem, FsOptions, ReadWriteSeek as _};
use crate::{
    bsp,
    config,
    error::{AppError, StorageError},
};
use embedded_sdmmc::{
    BlockDevice,
    Controller,
    Mode,
    TimeSource,
    filesystem,
    Timestamp,
    VolumeIdx,
    Attributes
};

// A dummy time source required by embedded-sdmmc.
pub struct DummyTimeSource;
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(1980, 1, 1, 0, 0, 0).unwrap()
    }
}

pub struct StorageManager<D>
where
    D: BlockDevice,
{
    fs: FileSystem<D>,
    file_counters: [u32; config::NUM_CAMERAS],
}

impl<D> StorageManager<D>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug, AppError: From<embedded_sdmmc::Error<<D as BlockDevice>::Error>> // Required by Controller methods
{
    /// Creates a new, uninitialized StorageManager.
    pub fn new(device: D) -> Self {
        let fs = FileSystem::new(device, FsOptions::new())
            .map_err(|e|)
        Self {
            fs: FileSystem::new(device, FsOptions::new())
            .map_err(|_| StorageError::VolumeAccessFailed)?,
            file_counters: [0; config::NUM_CAMERAS],
        }
    }


    /// Creates the necessary directories for each camera if they don't exist.
    pub fn prepare_directories(&mut self) -> Result<(), AppError> {
        let root_dir = self.fs.root_dir();
        for &dir_name in config::CAMERA_PREFIXES.iter() {
            defmt::info!("正在创建目录: {}", dir_name);
            match root_dir.create_dir_if_not_exists(dir_name) {
                Ok(_) => defmt::info!("目录 '{}' 已创建或已存在。", dir_name),
                Err(_e) => {
                    defmt::error!("创建目录 '{}' 失败: {:?}", dir_name, _e);
                    return Err(StorageError::DirectoryCreateFailed.into());
                }
            }
        }
        Ok(())
    }

    /// Saves an image buffer to the correct directory for the given camera ID.
    pub fn save_image(&mut self, cam_id: usize, image_data: &[u8]) -> Result<(), AppError> {
        use fatfs::ReadWriteSeek as _;

        if cam_id >= config::NUM_CAMERAS {
            return Err(StorageError::InvalidCameraId.into());
        }

        let dir_name = config::CAMERA_PREFIXES[cam_id];
        self.file_counters[cam_id] += 1;
        let file_num = self.file_counters[cam_id];

        let mut path_str = String::<20>::new();
        write!(
            &mut path_str,
            "/{}/{:04}.{}",
            dir_name,
            file_num,
            config::FILENAME_SUFFIX
        ).unwrap();

        let root_dir = self.fs.root_dir();
        // 创建文件
        let mut file = root_dir.create_file(path_str.as_str())
            .map_err(|_| StorageError::FileOpenFailed)?;

        // 写入数据
        file.write_all(image_data)
            .map_err(|_| StorageError::FileWriteFailed)?;

        // 文件在离开作用域时会自动关闭 (RAII)
        defmt::info!("图片已保存至: {}", path_str.as_str());
        Ok(())
    }
}