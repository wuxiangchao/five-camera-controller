#![no_std]
#![no_main]
#![allow(unused)]

use five_camera_controller as _;
use panic_probe as _;

/// Hard fault handler.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    defmt::error!("HARD FAULT");
    loop {
        cortex_m::asm::bkpt();
    }
}

#[rtic::app(
    device = stm32h7xx_hal::pac,
    dispatchers = [SPI1, SPI2, FPU]
)]
mod app {
    use five_camera_controller::{bsp, camera, config, fcu, storage, image, error::AppError, init_heap};
    use five_camera_controller::fcu::DronePose;

    use systick_monotonic::{Systick, ExtU64,};
    use systick_monotonic::fugit::RateExtU32;
    use five_camera_controller::bsp::SdCardDevice;
    use five_camera_controller::error::StorageError;

    #[shared]
    struct Shared {
        storage_manager: storage::StorageManager<SdCardDevice>,
        fcu_link: fcu::FcuLink,
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
        defmt::info!("Heap allocator initialized successfully");

        let board = bsp::setup(cx.device);
        let mono = Systick::new(cx.core.SYST, 400_000_000);
        let mut sdmmc = board.sdmmc;

        defmt::info!("Initializing SD Card...");
        // Loop until the card is initialized successfully
        loop {
            match sdmmc.init(25_u32.MHz()) {
                Ok(_) => {
                    defmt::info!("SD Card initialized successfully!");
                    break;
                }
                Err(_e) => {
                    defmt::error!("SD Card init error");
                    // In a real application, you might want a timeout here
                }
            }
        }

        // Now we can access the card size before wrapping it
        match sdmmc.card() {
            Ok(card) => defmt::info!("Card size: {} blocks", card.size()),
            Err(_) => defmt::warn!("Could not read card info"),
        }

        // Wrap the initialized device and create the StorageManager
        let total_blocks = sdmmc.card().unwrap().size();
        let block_device = sdmmc.sdmmc_block_device();
        let mut storage_manager =  match storage::StorageManager::new(block_device, total_blocks){
            Ok(sm) => sm,
            Err(e) => {
                defmt::error!("Failed to create StorageManager: {:?}", e);
                panic!("StorageManager init failed");
            }
        };
        let fcu_link = fcu::FcuLink::new(board.fcu_uart_rx, board.fcu_uart_tx);
        let camera_controller = camera::CameraController::new(board.triggers);


        defmt::info!("Preparing directories...");
        if let Err(e) = storage_manager.prepare_directories() {
            defmt::error!("Failed to prepare directories: {:?}", e);
            panic!("SD Card Error");
        }

        // 启动第一个周期性任务
        trigger_cameras::spawn().unwrap();

        defmt::info!("System Initialization Complete. Main loop starting.");

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

    ///
    #[task(local = [camera_controller], shared = [fcu_link])]
    fn trigger_cameras(mut cx: trigger_cameras::Context) {
        defmt::info!("Triggering all cameras...");

        let current_pose = cx.shared.fcu_link.lock(|fcu| fcu.get_latest_pose());

        cx.local.camera_controller.trigger_all();

        for i in 0..config::NUM_CAMERAS {
            process_image::spawn(current_pose, i).unwrap();
        }

        // 关键区别：任务执行完后，重新调度自己在一秒后再次运行
        trigger_cameras::spawn_after(1000.millis()).unwrap();
    }

    ///
    #[task(shared = [storage_manager], capacity = 5)]
    fn process_image(mut cx: process_image::Context, pose: Option<DronePose>, cam_id: usize) {
        defmt::info!("Processing image for camera ID: {}", cam_id);

        let mut dummy_image_data = [0u8; 1024];
        dummy_image_data[0] = cam_id as u8;

        if let Some(p) = pose {
            match image::embed_gps_metadata(&mut dummy_image_data, &p) {
                Ok(_) => defmt::info!("GPS metadata embedded for cam {}", cam_id),
                Err(_) => defmt::warn!("Failed to embed metadata for cam {}", cam_id),
            }
        }

        let result = cx.shared.storage_manager.lock(|sm| {
            sm.save_image(cam_id, &dummy_image_data)
        });

        if let Err(e) = result {
            defmt::error!("Failed to save image for cam {}: {:?}", cam_id, e);
        }
    }

    /// 硬件中断任务 - 这个任务的定义和之前完全一样
    #[task(binds = UART4, shared = [fcu_link])]
    fn uart_rx(mut cx: uart_rx::Context) {
        cx.shared.fcu_link.lock(|fcu| {
            fcu.process_incoming_data();
        });
    }
}