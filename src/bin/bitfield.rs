#![no_main]
#![no_std]

use cortex_m_rt::entry;
use five_camera_controller as _; // global logger ,panicking-behavior, memory layout
use stm32h7xx_hal::{delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    defmt::info!("Blink program started on STM32H743!");

    // 获取对核心外设和芯片外设的访问权
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // 时钟配置
    // 配置电源，获取对时钟的写权限
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    // 配置并使能时钟
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(400.MHz()).freeze(vos, &dp.SYSCFG);

    // GPIO 配置
    // 获取对GPIOC端口的控制权
    // TODO: 如果LED在其他端口, 比如GPIOB, 就改成 dp.GPIOB.split(ccdr.peripheral.GPIOB)
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // 将PC13引脚配置为推挽输出模式
    // TODO: 将 .pc13 改成实际的引脚, 比如 .pb7
    let mut led = gpioc.pc13.into_push_pull_output();

    // 延时器配置
    // 创建一个基于系统滴答定时器 (SysTick) 的延时器
    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    defmt::info!("Entering main loop...");

    // 主循环
    loop {
        defmt::debug!("Setting LED ON");
        // 将LED引脚设置为低电平来点亮LED (很多板子是低电平点亮)
        // 如果您是高电平点亮, 使用 .set_high()
        led.set_low();

        // 延时500毫秒
        delay.delay_ms(500_u16);

        defmt::debug!("Setting LED OFF");
        // 将LED引脚设置为高电平来熄灭LED
        led.set_high();

        // 延时500毫秒
        delay.delay_ms(500_u16);
    }
}
