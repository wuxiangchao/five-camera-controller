#![allow(unused)]

use core::fmt::Write as _;
use heapless::String;
use fatfs::{
    FileSystem, FsOptions, NullTimeProvider, ReadWriteSeek, Write,
    File as FatFile, LossyOemCpConverter
};
use crate::{
    config,
    error::{AppError, StorageError},
    bsp::SdCardDevice,
};
use embedded_sdmmc::BlockDevice;
mod sd_fat_wrapper;
use sd_fat_wrapper::SdFatWrapper;

type IoWrapper = &'static mut SdFatWrapper<SdCardDevice>;
type FatFs = FileSystem<IoWrapper, NullTimeProvider, LossyOemCpConverter>;
type File<'a> = FatFile<'a, IoWrapper, NullTimeProvider, LossyOemCpConverter>;

/// 高可靠FAT存储管理器
pub struct StorageManager<'a>
{
    fs: &'a FatFs,
    file_counters: [u32; config::NUM_CAMERAS],

    open_files: [Option<File<'a>>; config::NUM_CAMERAS],
}

impl<'a> StorageManager<'a>
{
    /// 初始化 FAT 文件系统
    pub fn new(
        device: SdCardDevice,
        total_blocks: u64
    ) -> Result<&'static FatFs, AppError> {
        static mut WRAPPER: Option<SdFatWrapper<SdCardDevice>> = None;
        static mut FS: Option<FatFs> = None;

        let wrapper = unsafe {
            WRAPPER.get_or_insert_with(|| SdFatWrapper::new(device, total_blocks))
        };

        let fs = unsafe {
            FS.get_or_insert_with(|| {
                FileSystem::new(wrapper, FsOptions::new())
                    .expect("Failed to create FileSystem")
            })
        };

        Ok(fs)
    }

    pub fn init(fs: &'a FatFs) -> Self {
        Self {
            fs,
            file_counters: [0; config::NUM_CAMERAS],
            // 初始化为空文件句柄
            open_files: [None, None, None, None, None],
        }
    }

    pub fn close_files(&mut self) -> Result<(), AppError> {
        for i in 0..config::NUM_CAMERAS {
            if let Some(mut file) = self.open_files[i].take() {
                file.flush().map_err(|_| StorageError::FileCloseFailed)?;
                defmt::info!("Closed file for CAM{}", i);
            }
        }
        Ok(())
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

    pub fn open_new_files(&mut self) -> Result<(), AppError> {
        // 先关闭所有旧文件
        self.close_files()?;

        let root = self.fs.root_dir();

        for cam_id in 0..config::NUM_CAMERAS {
            self.file_counters[cam_id] += 1;
            let file_num = self.file_counters[cam_id];
            let dir_name = config::CAMERA_PREFIXES[cam_id];

            let mut path = String::<32>::new();
            write!(
                &mut path,
                "/{}/{:04}.{}",
                dir_name,
                file_num,
                config::FILENAME_SUFFIX
            ).unwrap(); // 假设路径总是适合

            // 检查目录是否存在
            if root.open_dir(dir_name).is_err() {
                defmt::warn!("Directory {} does not exist, re-create.", dir_name);
                root.create_dir(dir_name)
                    .map_err(|_| StorageError::DirectoryCreateFailed)?;
            }

            // 创建新文件
            let file = root.create_file(path.as_str())
                .map_err(|e| {
                    defmt::error!("Create file {} failed", path.as_str());
                    StorageError::FileOpenFailed
                })?;

            defmt::info!("Opened new file: {}", path.as_str());
            self.open_files[cam_id] = Some(file);
        }
        Ok(())
    }

    /// 将数据块写入一个已打开的文件
    pub fn write_chunk(&mut self, cam_id: usize, chunk_data: &[u8]) -> Result<(), AppError> {
        if cam_id >= config::NUM_CAMERAS {
            return Err(StorageError::InvalidCameraId.into());
        }

        if let Some(file) = &mut self.open_files[cam_id] {
            file.write_all(chunk_data)
                .map_err(|_| StorageError::FileWriteFailed)?;
            Ok(())
        } else {
            defmt::error!("Attempted to write to closed file for CAM{}", cam_id);
            Err(StorageError::FileOpenFailed.into())
        }
    }
}