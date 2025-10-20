#![no_std]
#![no_main]
#![allow(unused)]

use five_camera_controller as _;

#[rtic::app(
    device = stm32h7xx_hal::pac,
    dispatchers = [SPI1, SPI2, FPU]
)]
mod app {
    use five_camera_controller::{
        bsp::{self, SdCardDevice},
        camera, config, error::AppError, fcu_link, image_processor, storage,
        init_heap
    };
    use five_camera_controller::fcu_link::DronePose;

    use rtic_monotonic::Monotonic;
    use systick_monotonic::{Systick, fugit::ExtU64};
    use systick_monotonic::fugit::RateExtU32;

    #[shared]
    struct Shared {
        storage_manager: storage::StorageManager<SdCardDevice>,
        fcu_link: fcu_link::FcuLink,
    }

    #[local]
    struct Local {
        camera_controller: camera::CameraController,
    }

    #[monotonic(binds = SysTick, default = true)]
    type AppMonotonic = Systick<1_000>; // 1000 Hz tick rate

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("System Initialization Started");

        init_heap();
        defmt::info!("Heap allocator initialized");

        let board = bsp::setup(cx.device);
        let mono = Systick::new(cx.core.SYST, 400_000_000);
        let mut sdmmc = board.sdmmc;

        defmt::info!("Initializing SD Card...");
        loop {
            match sdmmc.init(25_u32.MHz()) {
                Ok(_) => {
                    defmt::info!("SD Card initialized successfully!");
                    break;
                }
                Err(_) => {
                    defmt::error!("SD Card init error, retrying...");
                }
            }
        }

        match sdmmc.card() {
            Ok(card) => defmt::info!("Card size: {} blocks", card.size()),
            Err(_) => defmt::warn!("Could not read card info"),
        }

        let block_device = sdmmc.sdmmc_block_device();
        let mut storage_manager = storage::StorageManager::new(block_device);
        let fcu_link = fcu_link::FcuLink::new(board.fcu_uart_rx, board.fcu_uart_tx);
        let camera_controller = camera::CameraController::new(board.triggers);

        defmt::info!("Initializing filesystem volume...");
        if let Err(e) = storage_manager.init_volume() {
            defmt::error!("Failed to initialize volume: {:?}", e);
            panic!("Filesystem Error");
        }

        defmt::info!("Preparing storage...");
        if let Err(e) = storage_manager.prepare_directories() {
            defmt::error!("Failed to prepare directories: {:?}", e);
            panic!("Storage Error");
        }

        trigger_cameras::spawn().unwrap();

        defmt::info!("Initialization Complete.");

        (
            Shared {
                storage_manager,
                fcu_link,
            },
            Local {
                camera_controller,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [camera_controller], shared = [fcu_link])]
    fn trigger_cameras(mut cx: trigger_cameras::Context) {
        defmt::info!("Triggering all cameras...");
        let current_pose = cx.shared.fcu_link.lock(|fcu| fcu.get_latest_pose());
        cx.local.camera_controller.trigger_all();

        for i in 0..config::NUM_CAMERAS {
            process_image::spawn(current_pose, i).unwrap();
        }

        trigger_cameras::spawn_after(1000.millis()).unwrap();
    }

    #[task(shared = [storage_manager], capacity = 5)]
    fn process_image(mut cx: process_image::Context, pose: Option<DronePose>, cam_id: usize) {
        defmt::info!("Processing image for camera ID: {}", cam_id);
        let mut dummy_image_data = [0u8; 1024];
        dummy_image_data[0] = cam_id as u8;

        if let Some(p) = pose {
            if image_processor::embed_gps_metadata(&mut dummy_image_data, &p).is_err() {
                defmt::warn!("Failed to embed metadata for cam {}", cam_id);
            }
        }

        if let Err(e) = cx.shared.storage_manager.lock(|sm| {
            sm.save_image(cam_id, &dummy_image_data)
        }) {
            defmt::error!("Failed to save image for cam {}: {:?}", cam_id, e);
        }
    }

    #[task(binds = UART4, shared = [fcu_link])]
    fn uart_rx(mut cx: uart_rx::Context) {
        cx.shared.fcu_link.lock(|fcu| {
            fcu.process_incoming_data();
        });
    }
}
