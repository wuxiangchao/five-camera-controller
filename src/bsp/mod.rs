#![allow(unused)]
pub use stm32h7xx_hal::pac;
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{self, Output, PushPull},
    prelude::*,
    rtc::Rtc,
    rcc::rec,
    sdmmc::{SdCard, Sdmmc, SdmmcBlockDevice},
    serial::{Tx, Rx, Serial}, // <-- 我添加了 Serial，因为您在 setup 中使用了它
};

/// A type alias for the complex SD card block device type used by embedded-sdmmc.
pub type SdCardDevice = SdmmcBlockDevice<Sdmmc<pac::SDMMC1, SdCard>>;

/// A struct containing all the initialized peripherals for the board.
/// This is the return type of the `setup` function.
pub struct Board {
    /// The initialized SDMMC1 peripheral for SD card access.
    /// Note: `init_card` still needs to be called on this.
    pub sdmmc: Sdmmc<pac::SDMMC1, SdCard>,
    /// The five GPIO pins configured as outputs for triggering the cameras.
    pub triggers: CameraTriggers,
    /// The serial port (UART) for communication with the flight controller.
    pub fcu_serial: Serial<pac::UART4>, //
}

/// A struct to hold all five camera trigger pins, bundled for convenience.
pub struct CameraTriggers {
    pub cam1: gpio::PE11<Output<PushPull>>,
    pub cam2: gpio::PE12<Output<PushPull>>,
    pub cam3: gpio::PE13<Output<PushPull>>,
    pub cam4: gpio::PE14<Output<PushPull>>,
    pub cam5: gpio::PE15<Output<PushPull>>,
}

/// Initializes the entire board.
///
/// This function takes the core and device peripherals and returns a `Board`
/// struct containing all the initialized drivers.
pub fn setup(mut dp: pac::Peripherals) -> Board {
    // Clock and Power Configuration
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    // 时钟源
    let ccdr = rcc.sys_ck(400.MHz())
        .freeze(vos, &dp.SYSCFG);

    // GPIO Port Splitting
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // SDMMC Peripheral Initialization
    // These pins are configured based on your sdcard_test.rs file.
    let clk = gpioc.pc12.into_alternate().speed(gpio::Speed::VeryHigh);
    let cmd = gpiod
        .pd2
        .into_alternate()
        .internal_pull_up(true)
        .speed(gpio::Speed::VeryHigh);
    let d0 = gpioc
        .pc8
        .into_alternate()
        .internal_pull_up(true)
        .speed(gpio::Speed::VeryHigh);
    let d1 = gpioc
        .pc9
        .into_alternate()
        .internal_pull_up(true)
        .speed(gpio::Speed::VeryHigh);
    let d2 = gpioc
        .pc10
        .into_alternate()
        .internal_pull_up(true)
        .speed(gpio::Speed::VeryHigh);
    let d3 = gpioc
        .pc11
        .into_alternate()
        .internal_pull_up(true)
        .speed(gpio::Speed::VeryHigh);

    let sdmmc: Sdmmc<_, SdCard> = dp.SDMMC1.sdmmc(
        (clk, cmd, d0, d1, d2, d3),
        ccdr.peripheral.SDMMC1,
        &ccdr.clocks,
    );

    // Camera Trigger Pins Initialization ---
    let triggers = CameraTriggers {
        cam1: gpioe.pe11.into_push_pull_output(),
        cam2: gpioe.pe12.into_push_pull_output(),
        cam3: gpioe.pe13.into_push_pull_output(),
        cam4: gpioe.pe14.into_push_pull_output(),
        cam5: gpioe.pe15.into_push_pull_output(),
    };

    // FCU UART Initialization
    let tx = gpioa.pa0.into_alternate();
    let rx = gpioa.pa1.into_alternate();
    let fcu_serial = dp
        .UART4
        .serial((tx, rx), 115_200.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
        .unwrap();

    Board {
        sdmmc,
        triggers,
        fcu_serial,
    }
}