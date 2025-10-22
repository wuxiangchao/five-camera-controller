#![no_std]
#![no_main]
#![allow(unused, static_mut_refs)]
extern crate alloc;

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

    use alloc::vec;
    use alloc::vec::Vec;
    use stm32h7xx_hal::serial::Event as SerialEvent;
    use five_camera_controller::fcu::DequeReader;
    use mavio::io::EmbeddedIoReader;
    use mavio::{Receiver, error::IoError, protocol::Versionless};
    use heapless::spsc::{Queue, Producer, Consumer};
    use storage::StorageManager;

    use core::mem::MaybeUninit;
    use rtic::Monotonic;

    static mut STORAGE_MANAGER: MaybeUninit<StorageManager<'static>> = MaybeUninit::uninit();
    static mut CHUNK_CONSUMER: MaybeUninit<Consumer<'static, ImageChunk, IMAGE_CHUNK_QUEUE_SIZE>> = MaybeUninit::uninit();

    // 类型别名
    type ImageChunk = (usize, Vec<u8>, bool);
    const IMAGE_CHUNK_QUEUE_SIZE: usize = 16;
    type FcuDequeReader<'a> = DequeReader<'a, 1024>;
    type FcuReceiver<'a> = Receiver<IoError, EmbeddedIoReader<FcuDequeReader<'a>>, Versionless>;

    #[shared]
    struct Shared {
        fcu_byte_queue_prod: &'static mut Producer<'static, u8, 1024>,
        latest_pose: Option<DronePose>,
        chunk_queue_prod: &'static mut Producer<'static, ImageChunk, IMAGE_CHUNK_QUEUE_SIZE>,
    }

    //
    #[local]
    struct Local {
        camera_controller: camera::CameraController,
        fcu_link: fcu::FcuLink,
        fcu_receiver: FcuReceiver<'static>,
        storage_consumer_handle: storage_consumer::SpawnHandle,
    }

    #[monotonic(binds = SysTick, default = true)]
    type AppMonotonic = Systick<1_000>; // 1000 Hz tick rate

    //
    #[init(
        local = [
            // FCU队列
            fcu_queue: Queue<u8, 1024> = Queue::new(),
            fcu_prod: Option<Producer<'static, u8, 1024>> = None,
            fcu_cons: Option<Consumer<'static, u8, 1024>> = None,

            // 数据块队列
            chunk_queue: Queue<ImageChunk, IMAGE_CHUNK_QUEUE_SIZE> = Queue::new(),
            chunk_prod: Option<Producer<'static, ImageChunk, IMAGE_CHUNK_QUEUE_SIZE>> = None,
        ]
    )]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("System Initialization Started");
        init_heap();
        defmt::info!("Heap allocator initialized successfully");

        let board = bsp::setup(cx.device);
        let mut mono = Systick::new(cx.core.SYST, 400_000_000);
        let mut sdmmc = board.sdmmc;

        // SD卡初始化循环
        defmt::info!("Initializing SD Card...");
        let sd_init_timeout: <Systick<1000> as Monotonic>::Duration = 5_000.millis(); // 设置5秒超时
        let start_time = mono.now();
        loop {
            match sdmmc.init(25_u32.MHz()) {
                Ok(_) => { defmt::info!("SD Card initialized successfully!"); break; }
                Err(_e) => {
                    defmt::warn!("SD Card init error, retrying..."); //

                    // 检查是否超时
                    if mono.now() - start_time > sd_init_timeout {
                        defmt::error!("SD Card init TIMEOUT. Halting system.");
                        loop {
                            cortex_m::asm::bkpt(); //
                        }
                    }
                    cortex_m::asm::delay(100_000_000);
                }
            }
        }
        let total_blocks = sdmmc.card().unwrap().size();
        let block_device = sdmmc.sdmmc_block_device();
        let static_fs = StorageManager::new(block_device, total_blocks)
            .expect("Failed to init FileSystem");
        let mut storage_manager = StorageManager::init(static_fs);
        defmt::info!("Preparing directories...");
        storage_manager.prepare_directories().expect("Failed to prepare directories");

        // (FCU 队列初始化)
        let (fcu_tx, mut fcu_rx) = board.fcu_serial.split();
        fcu_rx.listen();
        let fcu_link = fcu::FcuLink::new(fcu_tx);
        let (fcu_prod, fcu_cons) = cx.local.fcu_queue.split();
        let static_fcu_prod = cx.local.fcu_prod.insert(fcu_prod);
        let static_fcu_cons = cx.local.fcu_cons.insert(fcu_cons);
        let deque_reader = FcuDequeReader::new(static_fcu_cons);
        let embedded_reader = EmbeddedIoReader::new(deque_reader);
        let fcu_receiver = Receiver::new(embedded_reader);

        // 数据块队列初始化
        let (chunk_prod, chunk_cons) = cx.local.chunk_queue.split();
        let static_chunk_prod = cx.local.chunk_prod.insert(chunk_prod);

        unsafe {
            STORAGE_MANAGER.write(storage_manager);
            CHUNK_CONSUMER.write(chunk_cons);
        }

        // 启动任务
        process_mavlink::spawn().unwrap();
        trigger_cameras::spawn().unwrap();
        let storage_consumer_handle = storage_consumer::spawn_after(10.millis()).unwrap();
        let camera_controller = camera::CameraController::new(board.triggers);

        defmt::info!("System Initialization Complete. Main loop starting.");

        //
        (
            Shared {
                fcu_byte_queue_prod: static_fcu_prod,
                latest_pose: None,
                chunk_queue_prod: static_chunk_prod,
            },
            Local {
                camera_controller,
                fcu_link,
                fcu_receiver,
                storage_consumer_handle,
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

    // 生产者 (Producer) 任务
    #[task(local = [camera_controller], shared = [latest_pose, chunk_queue_prod])]
    fn trigger_cameras(mut cx: trigger_cameras::Context) {
        defmt::info!("Triggering all cameras...");
        let current_pose = cx.shared.latest_pose.lock(|p| *p);
        cx.local.camera_controller.trigger_all();

        for cam_id in 0..config::NUM_CAMERAS {
            if let Err(_) = read_camera::spawn(cam_id, current_pose) {
                defmt::warn!("Failed to spawn read_camera for CAM{}", cam_id);
            }
        }
        trigger_cameras::spawn_after(1000.millis()).unwrap();
    }

    #[task(shared = [chunk_queue_prod], capacity = 5)]
    fn read_camera(mut cx: read_camera::Context, cam_id: usize, pose: Option<DronePose>) {
        defmt::info!("Reading from CAM{}...", cam_id);
        const TOTAL_CHUNKS: usize = 160;
        const CHUNK_SIZE: usize = 64 * 1024;

        for i in 0..TOTAL_CHUNKS {
            let mut chunk_data: Vec<u8> = vec![cam_id as u8; CHUNK_SIZE];
            if i == 0 {
                chunk_data[0] = 0xFF;
                chunk_data[1] = 0xD8;
                if let Some(p) = pose {
                    image::embed_gps_metadata(&mut chunk_data, &p).ok();
                }
            }
            let is_last_chunk = (i == TOTAL_CHUNKS - 1);
            let chunk_tuple = (cam_id, chunk_data, is_last_chunk);
            loop {
                match cx.shared.chunk_queue_prod.lock(|p| p.enqueue(chunk_tuple.clone())) {
                    Ok(_) => break,
                    Err(_) => {
                        defmt::warn!("Chunk queue full! CAM{} waiting...", cam_id);
                        cortex_m::asm::delay(1_000_000);
                    }
                }
            }
        }
        defmt::info!("Finished reading from CAM{}", cam_id);
    }

    #[task(
        capacity = 1
    )]
    fn storage_consumer(mut cx: storage_consumer::Context) {

        let storage_manager: &mut StorageManager = unsafe {
            STORAGE_MANAGER.assume_init_mut()
        };
        let chunk_consumer: &mut Consumer<ImageChunk, IMAGE_CHUNK_QUEUE_SIZE> = unsafe {
            CHUNK_CONSUMER.assume_init_mut()
        };

        let mut work_done = false;
        let mut open_files_for_cam = [false; config::NUM_CAMERAS];

        while let Some((cam_id, chunk_data, is_last_chunk)) =
            chunk_consumer.dequeue()
        {
            work_done = true;

            if !open_files_for_cam[cam_id] {
                defmt::info!("First chunk for CAM{}, opening file...", cam_id);
                storage_manager.open_new_files().ok();
                open_files_for_cam[cam_id] = true;
            }

            match storage_manager.write_chunk(cam_id, &chunk_data) {
                Ok(_) => defmt::trace!("Wrote chunk for CAM{}", cam_id),
                Err(e) => defmt::error!("Failed to write chunk for CAM{}: {:?}", cam_id, e),
            }

            if is_last_chunk {
                defmt::info!("Last chunk for CAM{}, closing file.", cam_id);
                storage_manager.close_files().ok();
                open_files_for_cam[cam_id] = false;
            }
        }

        if work_done {
            defmt::info!("Storage consumer finished batch, re-spawning.");
        }

        storage_consumer::spawn_after(10.millis()).unwrap();
    }

    #[task(binds = UART4, shared = [fcu_byte_queue_prod])]
    fn uart_rx(mut cx: uart_rx::Context) {
        let uart = unsafe { &*bsp::pac::UART4::ptr() };
        if uart.isr.read().rxne().bit_is_set() {
            let byte = uart.rdr.read().rdr().bits() as u8;
            cx.shared.fcu_byte_queue_prod.lock(|p| p.enqueue(byte).ok());
            process_mavlink::spawn().ok();
        }
    }

    #[task(local = [fcu_link, fcu_receiver], shared = [latest_pose], capacity = 1)]
    fn process_mavlink(mut cx: process_mavlink::Context) {
        cx.shared.latest_pose.lock(|pose| {
            cx.local.fcu_link.process_incoming_data(
                cx.local.fcu_receiver,
                pose
            );
        });
    }
}