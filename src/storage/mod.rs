#![allow(unused)]

use core::fmt::Write as _;
use heapless::String;
use fatfs::{FileSystem, FsOptions, NullTimeProvider, ReadWriteSeek, Write};
use crate::{
    config,
    error::{AppError, StorageError},
    bsp::SdCardDevice,
};
use embedded_sdmmc::BlockDevice;
mod sd_fat_wrapper;
use sd_fat_wrapper::SdFatWrapper;

/// 高可靠FAT存储管理器
pub struct StorageManager<D>
where
    D: BlockDevice,
{
    fs: FileSystem<SdFatWrapper<D>, NullTimeProvider>,
    file_counters: [u32; config::NUM_CAMERAS],
}

impl<D> StorageManager<D>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
{
    /// 初始化 FAT 文件系统
    pub fn new(device: D, total_blocks: u64) -> Result<Self, AppError> {
        let wrapper = SdFatWrapper::new(device, total_blocks);
        let fs = FileSystem::new(wrapper, FsOptions::new())
            .map_err(|_| StorageError::VolumeAccessFailed)?;
        Ok(Self {
            fs,
            file_counters: [0; config::NUM_CAMERAS],
        })
    }

    /// 确保相机目录存在，不存在则创建
    pub fn prepare_directories(&self) -> Result<(), AppError> {
        let root_dir = self.fs.root_dir();

        for &dir_name in config::CAMERA_PREFIXES.iter() {
            match root_dir.create_dir(dir_name) {
                Ok(_) => defmt::info!("Create directory {} success.", dir_name),
                Err(fatfs::Error::AlreadyExists) => {
                    defmt::info!("Directory {} already exist.", dir_name);
                }
                Err(_) => {
                    defmt::error!("Create directory: {} failed.", dir_name);
                    return Err(StorageError::DirectoryCreateFailed.into());
                }
            }
        }
        Ok(())
    }

    /// 保存图片数据到对应相机目录
    pub fn save_image(&mut self, cam_id: usize, image_data: &[u8]) -> Result<(), AppError> {
        if cam_id >= config::NUM_CAMERAS {
            return Err(StorageError::InvalidCameraId.into());
        }

        self.file_counters[cam_id] += 1;
        let file_num = self.file_counters[cam_id];
        let dir_name = config::CAMERA_PREFIXES[cam_id];

        // 拼接路径
        let mut path = String::<32>::new();

        write!(
            &mut path,
            "/{}/{:04}.{}",
            dir_name,
            file_num,
            config::FILENAME_SUFFIX
        ).unwrap();

        let root = self.fs.root_dir();

        // 检查目录是否存在
        match root.open_dir(dir_name) {
            Ok(_) => {}
            Err(_) => {
                defmt::warn!("Directory {} does not exist, re-create.", dir_name);
                root.create_dir(dir_name)
                    .map_err(|_| StorageError::DirectoryCreateFailed)?;
            }
        }

        let mut file = match root.create_file(&path) {
            Ok(f) => f,
            Err(fatfs::Error::AlreadyExists) => {
                defmt::warn!("File: {} already exist，write by overwrite...", path.as_str());
                root.open_file(&path)
                    .map_err(|_| StorageError::FileOpenFailed)?
            }
            Err(_) => {
                defmt::error!("Create file: {} failed", path.as_str());
                return Err(StorageError::FileOpenFailed.into());
            }
        };

        //
        file.write_all(image_data)
            .map_err(|_| StorageError::FileWriteFailed)?;

        file.flush().ok();
        defmt::info!("Image saved to: {}", path.as_str());
        Ok(())
    }
}
