#![allow(unused)]
use core::cmp::min;
use embedded_sdmmc::{blockdevice, BlockDevice, BlockIdx, Block};
use fatfs::{Error as FatError,IoError, Read, Write, Seek, SeekFrom, IoBase};

extern crate alloc_no_stdlib;
use alloc::vec::Vec;

const SECTOR_SIZE: usize = 512;
const WRITE_CACHE_SIZE_BLOCKS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WrapperError {
    /// 包装器内部的IO逻辑错误(如seek越界)
    IoError,
    /// 来自底层SD卡驱动的错误
    DeviceError,
}

impl IoError for WrapperError {
    fn is_interrupted(&self) -> bool {
        false
    }

    fn new_unexpected_eof_error() -> Self {
        WrapperError::IoError
    }

    fn new_write_zero_error() -> Self {
        WrapperError::IoError
    }
}

/// 将 embedded_sdmmc BlockDevice 适配为fatfs可读写设备
pub struct SdFatWrapper<D: BlockDevice> {
    inner: D,
    offset: u64,
    total_blocks: u64,

    read_buf: Block,
    // 大型写缓冲区(在AXISRAM堆上分配)
    write_cache: Vec<Block>,
}

impl<D: BlockDevice> SdFatWrapper<D> {
    pub fn new(inner: D, total_blocks: u64) -> Self {
        let mut write_cache = Vec::with_capacity(WRITE_CACHE_SIZE_BLOCKS);
        write_cache.resize_with(WRITE_CACHE_SIZE_BLOCKS, Block::new);

        Self {
            inner,
            offset: 0,
            total_blocks,
            read_buf: Block::new(), // 初始化读缓冲区
            write_cache, // 初始化写缓冲区
        }
    }

    fn read_block_into_buf(&mut self, block_idx: BlockIdx) -> Result<(), WrapperError> {
        // 直接读入self.read_buf
        self.inner.read(core::slice::from_mut(&mut self.read_buf), block_idx, "fatfs_read")
            .map_err(|_| {
                defmt::error!("SD Card read error");
                WrapperError::DeviceError
            })?;
        Ok(())
    }

    // 使用read_buf写入单个块
    fn write_block_from_buf(&mut self, block_idx: BlockIdx) -> Result<(), WrapperError> {
        // 克隆以允许在RMW循环中重复使用 read_buf
        self.inner.write(&[self.read_buf.clone()], block_idx)
            .map_err(|_| {
                defmt::error!("SD Card write error");
                WrapperError::DeviceError
            })?;
        Ok(())
    }
}

impl<D: BlockDevice> IoBase for SdFatWrapper<D>
where
    D::Error: core::fmt::Debug,
{
    type Error = WrapperError;
}

impl<D: BlockDevice> Read for SdFatWrapper<D>
where
    D::Error: core::fmt::Debug,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, WrapperError> {
        let mut bytes_read = 0;
        let mut remaining = buf.len();

        while remaining > 0 {
            let block_idx = self.offset / SECTOR_SIZE as u64;
            let block_offset = (self.offset % SECTOR_SIZE as u64) as usize;
            if block_idx >= self.total_blocks {
                break;
            }
            self.read_block_into_buf(BlockIdx(block_idx as u32))?;
            let to_copy = min(SECTOR_SIZE - block_offset, remaining);
            buf[bytes_read..bytes_read + to_copy]
                .copy_from_slice(&self.read_buf[block_offset..block_offset + to_copy]);
            self.offset += to_copy as u64;
            bytes_read += to_copy;
            remaining -= to_copy;
        }

        Ok(bytes_read)
    }
}

impl<D: BlockDevice> Write for SdFatWrapper<D>
where
    D::Error: core::fmt::Debug,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let aligned_start = self.offset % SECTOR_SIZE as u64 == 0;
        let aligned_end = buf.len() % SECTOR_SIZE == 0;

        // 如果写入是对齐的(在512B边界上开始)并且长度是 512B 的倍数,
        // 可以跳过RMW并执行高速的多块写入。
        if aligned_start && aligned_end && buf.len() > 0 {
            let num_blocks = buf.len() / SECTOR_SIZE;
            let start_block_idx = (self.offset / SECTOR_SIZE as u64) as u32;

            let mut blocks_written = 0;

            // 以64KB(CACHE_SIZE_BLOCKS)为单位循环写入
            while blocks_written < num_blocks {
                //计算本次循环要写入多少块
                let blocks_to_write_this_loop = min(
                    num_blocks - blocks_written,
                    WRITE_CACHE_SIZE_BLOCKS
                );

                // 确定要写入的write_cache切片
                let cache_slice = &mut self.write_cache[0..blocks_to_write_this_loop];

                // 将数据从buf复制到write_cache
                for i in 0..blocks_to_write_this_loop {
                    let buf_start = (blocks_written + i) * SECTOR_SIZE;
                    let buf_end = buf_start + SECTOR_SIZE;
                    cache_slice[i].as_mut_slice().copy_from_slice(&buf[buf_start..buf_end]);
                }

                // 执行多块写入
                let current_block_idx = BlockIdx(start_block_idx + blocks_written as u32);
                self.inner.write(cache_slice, current_block_idx)
                    .map_err(|_| {
                        defmt::error!("SD Card multi-block write error");
                        WrapperError::DeviceError
                    })?;

                blocks_written += blocks_to_write_this_loop;
            }

            self.offset += buf.len() as u64;
            Ok(buf.len())
        }

        // 这用于处理未对齐的写入(例如文件开头/结尾或元数据更新)
        else {
            let mut bytes_written_slow = 0;
            let mut remaining_slow = buf.len();

            while remaining_slow > 0 {
                let block_idx_u64 = self.offset / SECTOR_SIZE as u64;
                let block_offset = (self.offset % SECTOR_SIZE as u64) as usize;
                if block_idx_u64 >= self.total_blocks {
                    break;
                }

                let block_idx = BlockIdx(block_idx_u64 as u32);

                // RMW: Read
                self.read_block_into_buf(block_idx)?;

                let to_copy = min(SECTOR_SIZE - block_offset, remaining_slow);

                // RMW: Modify
                self.read_buf.as_mut_slice()[block_offset..block_offset + to_copy]
                    .copy_from_slice(&buf[bytes_written_slow..bytes_written_slow + to_copy]);

                // RMW: Write
                self.write_block_from_buf(block_idx)?;

                self.offset += to_copy as u64;
                bytes_written_slow += to_copy;
                remaining_slow -= to_copy;
            }
            Ok(bytes_written_slow)
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<D: BlockDevice> Seek for SdFatWrapper<D> {
    fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        let new_pos = match pos {
            SeekFrom::Start(off) => off as i64,
            SeekFrom::Current(off) => self.offset as i64 + off,
            SeekFrom::End(off) => {
                let total_size = self.total_blocks * SECTOR_SIZE as u64;
                total_size as i64 + off
            }
        };

        if new_pos < 0 {
            return Err(WrapperError::IoError);
        }

        self.offset = new_pos as u64;
        Ok(self.offset)
    }
}
