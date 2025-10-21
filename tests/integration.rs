#![no_std]
#![no_main]

use defmt_rtt as _; // 日志输出
use panic_probe as _; // 恐慌处理
use five_camera_controller as _; // 确保您的库被链接

#[defmt_test::tests]
mod tests {
    // 您可以在这里使用您的库
    // use five_camera_controller::some_function;

    #[test]
    fn integration_test_one() {
        // 在这里写您的集成测试逻辑
        defmt::info!("Running integration test...");
        assert!(true);
    }
}