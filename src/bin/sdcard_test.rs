#![no_main]
#![no_std]

use cortex_m_rt::entry;
use five_camera_controller as _;

// 正确的导入
use embedded_sdmmc::{Controller, Mode, TimeSource, Timestamp, VolumeIdx};
use stm32h7xx_hal::{
    gpio::Speed,
    pac,
    prelude::*,
    sdmmc::{SdCard, Sdmmc}, // 需要导入 SdCard和Sdmmc
};

// 时间源
pub struct DummyTimeSource;
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[entry]
fn main() -> ! {
    defmt::info!("SD Card test based on official example started!");

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // HAL硬件初始化
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(400.MHz()).freeze(vos, &dp.SYSCFG);
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // 根据板定义修改引脚配置
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let clk = gpioc.pc12.into_alternate().speed(Speed::VeryHigh);
    let cmd = gpiod
        .pd2
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d0 = gpioc
        .pc8
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d1 = gpioc
        .pc9
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d2 = gpioc
        .pc10
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d3 = gpioc
        .pc11
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);

    // 创建SDMMC对象
    let mut sdmmc: Sdmmc<_, SdCard> = dp.SDMMC1.sdmmc(
        (clk, cmd, d0, d1, d2, d3),
        ccdr.peripheral.SDMMC1,
        &ccdr.clocks,
    );

    // 使用HAL的方法初始化SD卡
    loop {
        match sdmmc.init(25.MHz()) {
            // 从一个较稳定的频率开始
            Ok(_) => {
                defmt::info!("SD Card initialized successfully!");
                break;
            }
            Err(e) => {
                match e {
                    stm32h7xx_hal::sdmmc::Error::Timeout => {
                        defmt::error!("Timeout while waiting for response.")
                    }
                    stm32h7xx_hal::sdmmc::Error::SoftwareTimeout => {
                        defmt::error!("Software timeout.")
                    }
                    stm32h7xx_hal::sdmmc::Error::Crc => defmt::error!("CRC error."),
                    stm32h7xx_hal::sdmmc::Error::UnsupportedCardType => {
                        defmt::error!("Unsupported card type.")
                    }
                    stm32h7xx_hal::sdmmc::Error::UnsupportedCardVersion => {
                        defmt::error!("Unsupported card version.")
                    }
                    _ => defmt::error!("Unknown error"),
                }
                defmt::info!("Retrying...");
                delay.delay_ms(1000_u32);
            }
        }
    }
    defmt::info!("Card size: {} blocks", sdmmc.card().unwrap().size());

    // 使用HAL提供的兼容性方法连接到embedded-sdmmc
    // 调用.sdmmc_block_device()获取包装器！
    let block_device = sdmmc.sdmmc_block_device();
    let mut controller = Controller::new(block_device, DummyTimeSource);

    let mut volume = match controller.get_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            match e {
                embedded_sdmmc::Error::EndOfFile => defmt::error!("End of file reached."),
                embedded_sdmmc::Error::ReadOnly => defmt::error!("Volume is read-only."),
                embedded_sdmmc::Error::NoSuchVolume => defmt::error!("No such volume."),
                embedded_sdmmc::Error::Unsupported => defmt::error!("Unsupported operation."),
                embedded_sdmmc::Error::NotEnoughSpace => {
                    defmt::error!("Not enough space on the volume.")
                }
                _ => defmt::error!("Unknown error"),
            }
            loop {}
        }
    };

    let root_dir = controller.open_root_dir(&volume).unwrap();

    let mut file = controller
        .open_file_in_dir(&mut volume, &root_dir, "hello.txt", Mode::ReadWriteCreate)
        .unwrap();

    controller
        .write(
            &mut volume,
            &mut file,
            b"This is the first line written using embedded-sdmmc!\n",
        )
        .unwrap();

    controller.close_file(&volume, file).unwrap();
    controller.close_dir(&volume, root_dir);
    defmt::info!("File write and close successful. Test finished. ✅");

    loop {
        cortex_m::asm::wfi();
    }
}
