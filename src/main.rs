#![no_std]
#![no_main]
#![allow(unused)]
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

    // 2. 为 FCU/ISR 修复添加的 use
    use alloc::vec; // 用于 Vec<u8>
    use alloc::vec::Vec; // 用于 Vec<u8>
    use stm32h7xx_hal::serial::Event as SerialEvent;
    use five_camera_controller::fcu::DequeReader;
    use mavio::io::EmbeddedIoReader;
    use mavio::{Receiver, error::IoError, protocol::Versionless};
    use heapless::spsc::{Queue, Producer, Consumer};

    // 3. 为 FCU/ISR 修复添加的类型别名
    type FcuDequeReader<'a> = DequeReader<'a, 1024>;
    type FcuReceiver<'a> = Receiver<IoError, EmbeddedIoReader<FcuDequeReader<'a>>, Versionless>;

    #[shared]
    struct Shared {
        storage_manager: storage::StorageManager<SdCardDevice>,
        fcu_byte_queue_prod: &'static mut Producer<'static, u8, 1024>,
        latest_pose: Option<DronePose>,
    }

    #[local]
    struct Local {
        camera_controller: camera::CameraController,
        fcu_link: fcu::FcuLink,
        fcu_receiver: FcuReceiver<'static>,
        // fcu_byte_queue_cons: &'static mut Consumer<'static, u8, 1024>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type AppMonotonic = Systick<1_000>; // 1000 Hz tick rate

    #[init(
        local = [
            queue: Queue<u8, 1024> = Queue::new(),
            prod: Option<Producer<'static, u8, 1024>> = None,
            cons: Option<Consumer<'static, u8, 1024>> = None,
        ]
    )]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("System Initialization Started");

        init_heap();
        defmt::info!("Heap allocator initialized successfully");

        let board = bsp::setup(cx.device);
        let mono = Systick::new(cx.core.SYST, 400_000_000);
        let mut sdmmc = board.sdmmc;

        defmt::info!("Initializing SD Card...");
        // 保持 SD 卡初始化循环 (如前所述，可添加超时)
        loop {
            match sdmmc.init(25_u32.MHz()) {
                Ok(_) => {
                    defmt::info!("SD Card initialized successfully!");
                    break;
                }
                Err(_e) => {
                    defmt::error!("SD Card init error");
                }
            }
        }
        match sdmmc.card() {
            Ok(card) => defmt::info!("Card size: {} blocks", card.size()),
            Err(_) => defmt::warn!("Could not read card info"),
        }

        let total_blocks = sdmmc.card().unwrap().size();
        let block_device = sdmmc.sdmmc_block_device();
        let mut storage_manager =  match storage::StorageManager::new(block_device, total_blocks){
            Ok(sm) => sm,
            Err(e) => {
                defmt::error!("Failed to create StorageManager: {:?}", e);
                panic!("StorageManager init failed");
            }
        };

        defmt::info!("Preparing directories...");
        if let Err(e) = storage_manager.prepare_directories() {
            defmt::error!("Failed to prepare directories: {:?}", e);
            panic!("SD Card Error");
        }

        // FCU 和 ISR 修复的初始化
        let (fcu_tx, mut fcu_rx) = board.fcu_serial.split();
        fcu_rx.listen();
        let fcu_link = fcu::FcuLink::new(fcu_tx);

        // 设置 'static 队列
        // let mut queue = Queue::<u8, 1024>::new();
        let (prod, cons) = cx.local.queue.split();
        let static_prod = cx.local.prod.insert(prod);
        let static_cons = cx.local.cons.insert(cons);

        let deque_reader = FcuDequeReader::new(static_cons);
        let embedded_reader = EmbeddedIoReader::new(deque_reader);
        let fcu_receiver = Receiver::new(embedded_reader);

        // 启动任务
        process_mavlink::spawn().unwrap(); // 启动MAVLink解析任务
        trigger_cameras::spawn().unwrap(); // 启动相机触发任务

        let camera_controller = camera::CameraController::new(board.triggers);

        defmt::info!("System Initialization Complete. Main loop starting.");

        (
            Shared {
                storage_manager,
                fcu_byte_queue_prod: static_prod,
                latest_pose: None,
            },
            Local {
                camera_controller,
                fcu_link,
                fcu_receiver,
                // fcu_byte_queue_cons: static_cons,
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

    /// 触发相机并 spawn *一个* 序列处理任务
    #[task(local = [camera_controller], shared = [latest_pose])]
    fn trigger_cameras(mut cx: trigger_cameras::Context) {
        defmt::info!("Triggering all cameras...");

        let current_pose = cx.shared.latest_pose.lock(|p| *p);

        cx.local.camera_controller.trigger_all();

        // 移除 for 循环，只 spawn 一个任务
        if let Err(_) = process_sequence::spawn(current_pose) {
            defmt::warn!("Process sequence task queue is full!");
        }

        // 重新调度自己
        trigger_cameras::spawn_after(1000.millis()).unwrap();
    }

    /// 串行处理和保存所有图像
    #[task(shared = [storage_manager], capacity = 1)]
    fn process_sequence(mut cx: process_sequence::Context, pose: Option<DronePose>) {
        defmt::info!("Starting image processing sequence...");

        // 将 for 循环移到此任务内部
        for cam_id in 0..config::NUM_CAMERAS {
            defmt::info!("Processing image for camera ID: {}", cam_id);

            // 使用堆分配的 Vec<u8>，而不是栈数组
            let mut image_data_vec: Vec<u8> = vec![0u8; 1024]; // 模拟 1KB 图像
            image_data_vec[0] = cam_id as u8;

            if let Some(p) = pose {
                match image::embed_gps_metadata(&mut image_data_vec, &p) {
                    Ok(_) => defmt::info!("GPS metadata embedded for cam {}", cam_id),
                    Err(_) => defmt::warn!("Failed to embed metadata for cam {}", cam_id),
                }
            }

            let result = cx.shared.storage_manager.lock(|sm| {
                sm.save_image(cam_id, &image_data_vec) // 传入 slice
            });

            if let Err(e) = result {
                defmt::error!("Failed to save image for cam {}: {:?}", cam_id, e);
            }
        }
        defmt::info!("Image processing sequence complete.");
    }

    /// 硬件中断任务 - 仅读取字节并推入队列
    #[task(binds = UART4, shared = [fcu_byte_queue_prod])]
    fn uart_rx(mut cx: uart_rx::Context) {
        // 安全地访问硬件寄存器
        let uart = unsafe { &*bsp::pac::UART4::ptr() };

        if uart.isr.read().rxne().bit_is_set() {
            // 读取数据寄存器会自动清除中断标志
            let byte = uart.rdr.read().rdr().bits() as u8;

            // 推入队列，如果满了则忽略 (丢弃字节)
            cx.shared.fcu_byte_queue_prod.lock(|p| {
                p.enqueue(byte).ok();
            });

            // 唤醒解析任务
            process_mavlink::spawn().ok();
        }
    }

    /// MAVLink 解析软件任务
    #[task(local = [fcu_link, fcu_receiver], shared = [latest_pose], capacity = 1)]
    fn process_mavlink(mut cx: process_mavlink::Context) {
        // 在此软件任务中安全地调用阻塞的recv()
        cx.shared.latest_pose.lock(|pose| {
            cx.local.fcu_link.process_incoming_data(
                cx.local.fcu_receiver,
                pose
            );
        });
    }
}