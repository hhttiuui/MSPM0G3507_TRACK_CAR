# MSPM0G3507 四轮小车

电赛（电子设计竞赛）的四轮小车整车控制程序，跑在 TI **MSPM0G3507**
（Arm Cortex-M0+，80 MHz）上，NoRTOS 裸机，1 ms 定时器分槽调度。

硬件是四路电机（M1~M4）、两路编码器（M1/M3）、IMU660RB 姿态模块、
SSD1306 OLED、四个按键、舵机、蜂鸣器，以及一个 DX-NR02 / Si24R1
2.4G 无线模块。

```
IMU660RB ──SPI──▶ Fusion AHRS ──▶ pitch / yaw
                                    │
              航向环（保持直行）  ──▶ 舵机 PWM (PA28)
                                    │
   循迹传感器 ──UART0──▶ 循迹 PID ──▶ 速度环（20ms）──▶ 四路电机 PWM
                                    │
                          SSD1306 OLED 菜单 + 4 键
```

## 硬件平台

| 项目 | 说明 |
|---|---|
| MCU | TI **MSPM0G3507**，Arm Cortex-M0+，80 MHz |
| 开发板 | LP-MSPM0G3507 LaunchPad |
| 姿态 | 逐飞 IMU660RB（LSM6DSR + Fusion AHRS，SPI） |
| 显示 | SSD1306 OLED，硬件 I2C |
| 电机 | 四路直流电机，每路 PWM + 两路方向脚 |
| 编码器 | M1 / M3 两路正交编码器（A 相 GPIO 中断 + B 相捕获） |
| 无线 | DX-NR02 / Si24R1（NRF24L01 兼容），软件 SPI |
| 开发环境 | CCS（Theia 版）70.5.0 + MSPM0 SDK **2.10.00.04** |

## 调度结构

主循环只跑非实时任务：

```c
while (1) {
    App_ProcessTrackUart();   // 循迹串口回传
    Menu_HandleKeys();        // 菜单按键
    Menu_Draw();              // OLED 刷新
}
```

实时任务全在 `TIMER_0` 的 1 ms 中断里，五个槽循环（整轮 5 ms）：

| 槽位 | 周期 | 内容 |
|---|---|---|
| 0 | 5 ms | `encoder_task()` + `Read_IMU660RB()` + `AngleServo_Update()` + `VehicleYaw_Update5ms()` |
| 1 | 5 ms | `FusionTasks()` —— AHRS 姿态解算 |
| 2 | 5 ms | 有编码器样本时执行 `HeadingControl_Update20ms()` + `SpeedControl_Update20ms()` |
| 3 | 5 ms | `ModeExecTask()` —— 按 `g_menuMode` 分派到五种运行模式 |
| 4 | 5 ms | 空闲 |

按键 `Key_Update1ms()` 每 1 ms 采样消抖。`SysTick_Handler` 只维护 `tick_ms` 计时。

## 五种运行模式

菜单里选 `MODE1`~`MODE5`，`START` 启动，`RESET` 急停并重置 AHRS：

| 模式 | 功能 |
|---|---|
| **MODE1** | 航向保持直行。目标速度 20（脉冲/20ms），航向环自动修正偏航 |
| **MODE2** | 正方形循迹。状态机 NORMAL → ENTRY_WAIT → TURN_L/R → FINISH，走完设定直角数后蜂鸣 1 秒停车 |
| **MODE3** | 空壳（TODO） |
| **MODE4** | 空壳（TODO） |
| **MODE5** | 循迹 + 定距双段停车。`stopDistance` 到点冻结计时，`stopDistance2` 到点物理停车 |

MODE2 的转弯判据：左/右侧三个传感器中 **≥2 个**检测到黑线即进入转弯；
中心两个传感器同时检测到黑线即退出转弯。

## 四轮速度控制

| 车轮 | 控制方式 | 说明 |
|---|---|---|
| M1 右后 | 闭环 | 使用 M1 编码器反馈 |
| M2 右前 | 跟随 | 跟随 M1 输出，静态补偿 `+75‰` |
| M3 左后 | 闭环 | 使用 M3 编码器反馈 |
| M4 左前 | 跟随 | 跟随 M3 输出，静态补偿 `-75‰` |

当前固化参数（`hardware/speed_control_config.h`）：

- `Kp = 30.0`、`Ki = 1.5`（已含 20 ms 控制周期）、`Kd = 0.0`
- PWM 输出上限 `800/1000`
- 目标斜坡 `1 pulse / 20ms`
- 正向前馈按 Target 8~20 标定
- 包含反向积分卸载与抗积分饱和回算

静态补偿只在左右两侧目标都为正时生效。航向的动态偏差应由 IMU 航向外环
修正，不要靠改速度环来替代。

### 正式接口

```c
SpeedControl_Init();
SpeedControl_SetTargets(leftTarget, rightTarget);
SpeedControl_Start();

/* 每 20ms 编码器采样后调用 */
SpeedControl_Update20ms(leftActual, rightActual);

SpeedControl_Stop();
```

上电后速度环默认停止，由菜单或外环通过上面三个接口控制。

## 2.4G 无线

`hardware/nr02_spi.c` 提供软件 SPI 位操作层，`hardware/si24r1.c` 是
Si24R1（NRF24L01 兼容）收发驱动，移植自 DX-NR02 的 STM32 示例，
**去掉了 IRQ 引脚依赖**，改为轮询 + 超时。

| 信号 | 引脚 |
|---|---|
| CSN | PB12 |
| CE | PB6 |
| SCK | PB8 |
| MOSI | PB7 |
| MISO | PB9 |

## 参数存储

`hardware/param_storage.c` 用 TI 的 **EEPROM 仿真（Type A）** 把各模式的
PID 参数、目标速度、停车距离等存进 Flash，上电由 `Param_Init()` 恢复。
菜单里改完参数需要执行保存才会写 Flash。

## 主要引脚

全部来自 `gpio_toggle_output.syscfg`：

| 功能 | 引脚 |
|---|---|
| KEY1 / KEY2 / KEY3 / KEY4 | PA2 / PB1 / PB17 / PB20 |
| OLED SDA / SCL | PA16 / PA15 |
| M1 编码器 A / B | PA13 / PA12 |
| M3 编码器 A / B | PB5 / PB4 |
| IMU SPI SCLK / MOSI / MISO / CS | PA17 / PB22 / PB14 / PB0 |
| 电机 PWM（PWM_0） | PA21 / PA27 |
| 电机 PWM（PWM_1） | PB26 / PB27 |
| 电机方向 AIN1/AIN2/BIN1/BIN2/CIN1/CIN2/DIN1/DIN2 | PB13 / PA14 / PB18 / PB19 / PA25 / PB25 / PA1 / PA26 |
| 舵机 PWM | PA28（TIMG7） |
| 蜂鸣器 | PB11 |
| LED | PA7 |
| 循迹 UART0 TX / RX | PA8 / PA9，115200 |
| 2.4G 模块 | PB12 / PB6 / PB8 / PB7 / PB9 |

## 目录

```
gpio_toggle_output.c          初始化、1ms 调度、中断、主循环
gpio_toggle_output.syscfg     外设与引脚配置（唯一配置来源）
小车代码使用说明书.md          13 章操作手册：菜单、模式、调参、故障排查
菜单系统执行报告.md            菜单系统设计说明
hardware/
  ttop_funtion.c              五种运行模式的分派与实现
  speed_control.c             速度环 V2
  speed_control_config.h      速度环统一调参入口
  heading_control.c           航向环
  vehicle_yaw.c               偏航角处理
  angle_servo_bridge.c        俯仰角 → 舵机输出
  servo.c / motor.c / encoder.c
  track.c                     循迹 PID
  top_zhengfangxing.c         正方形循迹状态机
  nr02_spi.c / si24r1.c       2.4G 无线
  menu.c / key.c / oled_hardware_i2c.c
  param_storage.c             Flash 参数存储（EEPROM 仿真）
  buzzer.c / clock.c / delay.c
  IMU660RB/                   IMU 驱动 + Fusion AHRS
test_results/                 已完成测试的原始数据归档，不参与固件编译
targetConfigs/                调试器配置
```

## 编译

需要 **CCS**（Theia 版）和 **MSPM0 SDK 2.10.00.04**。

1. CCS → `File → Import → CCS Projects`，选本目录
2. 确认 `Properties → General → MSPM0 SDK` 指向 2.10.00.04
3. `Build Project`

命令行方式：

```powershell
C:\ti\ccs2041\ccs\utils\bin\gmake.exe -C Debug -j 16 all
```

首次编译会自动生成 `Debug/`，不用管。

## 踩过的坑

- **`.gitignore` 别把工程文件也忽略掉。** 之前有一版把 `*.ccsproject`、
  `*.cproject`、`*.project`、`.settings/` 全忽略了，别人 clone 下来打不开工程。
  这些是**必须入库**的。
- **`Debug/` 不要提交。** 445 个文件、8 MB，每次编译全变。
- **`.vscode/c_cpp_properties.json` 写死了本机 SDK 路径**（`C:/ti/...`），
  换机器就失效，所以不入库。
- **`README.html` 是 TI 例程的自动生成文档**，描述的是 GPIO 翻转例程，
  和现在的代码无关，已删除。
- **工程名还叫 `gpio_toggle_output`。** 是从 TI 例程改建的，改名会牵动
  `.ccsproject` / `.cproject` / `.syscfg` 多处引用，风险大于收益，保留原名。

## 许可

[MIT](LICENSE)。第三方组件（TI SDK、ST 的 LSM6DSR 驱动、Fusion AHRS）
的许可见 [NOTICE.md](NOTICE.md)。
