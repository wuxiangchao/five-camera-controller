#![allow(unused)]
use core::cmp::min;
use embedded_sdmmc::{blockdevice, BlockDevice, BlockIdx};
use fatfs::{Error as FatError,IoError, Read, Write, Seek, SeekFrom, IoBase};
use embedded_sdmmc::Block;

const SECTOR_SIZE: usize = 512;

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
    block_buf: [u8; SECTOR_SIZE],
}

impl<D: BlockDevice> SdFatWrapper<D> {
    pub fn new(inner: D, total_blocks: u64) -> Self {
        Self {
            inner,
            offset: 0,
            total_blocks,
            block_buf: [0u8; SECTOR_SIZE],
        }
    }

    fn read_block(&mut self, block_idx: BlockIdx) -> Result<(), WrapperError> {
        let mut block = [Block::new()];
        self.inner.read(&mut block, block_idx, "fatfs_read")
            .map_err(|_| {
                defmt::error!("SD Card read error");
                WrapperError::DeviceError
            })?;
        ;
        self.block_buf.copy_from_slice(block[0].as_slice());
        Ok(())
    }

    fn write_block(&mut self, block_idx: BlockIdx) -> Result<(), WrapperError> {
        use embedded_sdmmc::Block;
        let mut block = Block::new();
        block.as_mut_slice().copy_from_slice(&self.block_buf);
        self.inner.write(&[block], block_idx)
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
            self.read_block(BlockIdx(block_idx as u32))?;
            let to_copy = min(SECTOR_SIZE - block_offset, remaining);
            buf[bytes_read..bytes_read + to_copy]
                .copy_from_slice(&self.block_buf[block_offset..block_offset + to_copy]);
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
        let mut bytes_written = 0;
        let mut remaining = buf.len();

        while remaining > 0 {
            let block_idx_u64 = self.offset / SECTOR_SIZE as u64;
            let block_offset = (self.offset % SECTOR_SIZE as u64) as usize;
            if block_idx_u64 >= self.total_blocks {
                break;
            }

            let block_idx = BlockIdx(block_idx_u64 as u32);
            self.read_block(block_idx)?;
            let to_copy = min(SECTOR_SIZE - block_offset, remaining);
            self.block_buf[block_offset..block_offset + to_copy]
                .copy_from_slice(&buf[bytes_written..bytes_written + to_copy]);
            self.write_block(block_idx)?;
            self.offset += to_copy as u64;
            bytes_written += to_copy;
            remaining -= to_copy;
        }
        Ok(bytes_written)
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
