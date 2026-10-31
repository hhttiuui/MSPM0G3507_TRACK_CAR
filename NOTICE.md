# 第三方组件与许可声明

本仓库的**自研部分**（`gpio_toggle_output.c`、`gpio_toggle_output.syscfg`、
`hardware/` 下除 `IMU660RB/` 外的代码、各 `.md` 文档）由 hhttiuui 编写，
以 [MIT](LICENSE) 发布。

选 MIT 的依据：自研代码不含任何 GPL 组件，用到的第三方内容要么是宽松许可
（MIT / BSD 系），要么根本不在仓库里（TI SDK）。下面逐项说明。

---

## 1. TI MSPM0 SDK — TI 商业软件许可

本仓库**不包含 SDK 源码**。编译时需要自行安装
**MSPM0 SDK 2.10.00.04**，并遵守 TI 的软件许可协议
（`license_mspm0_sdk_2_10_00_04.txt`，随 SDK 分发）。

要点：
- SDK 可免费用来开发 TI 器件；
- 编译产物（Output）可自由分发；
- SDK 源码本身不应原样再分发。

所以仓库里没有 `startup_mspm0g350x_ticlang.c`、`ti/driverlib/*` 这些文件
——它们来自 SDK，构建时自动引用。

---

## 2. SysConfig 生成文件

`Debug/ti_msp_dl_config.c` / `.h` 由 TI SysConfig 根据
`gpio_toggle_output.syscfg` 自动生成，属于 TI 许可里的 "Output"。

它们**不在本仓库里**（`Debug/` 已加入 `.gitignore`），首次 Build 时自动重建。
要改引脚或外设配置，请改 `.syscfg`，不要改生成文件。

---

## 3. STMicroelectronics LSM6DSR 驱动 — ST 版权

路径：`hardware/IMU660RB/lsm6dsr_reg.c`、`lsm6dsr_reg.h`

```
Copyright (c) 2021 STMicroelectronics. All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
```

本仓库**未附带** ST 的那份 LICENSE 文件（上游组件包里才有），
因此按文件头声明以 **AS-IS** 提供。如果你要把这段代码用于商业产品，
建议回 ST 官方仓库确认当前许可条款（ST 的 MEMS 驱动通常采用
BSD-3-Clause）。

文件头的版权声明请勿删除。

---

## 4. Fusion AHRS — MIT

路径：`hardware/IMU660RB/Fusion/`

```
The MIT License (MIT)

Copyright (c) 2021 x-io Technologies
```

完整许可证见 `hardware/IMU660RB/Fusion/LICENSE.md`。MIT 与 MIT 兼容，
可以随本项目一起按 MIT 分发。

---

## 5. 逐飞 IMU660RB 模块驱动

路径：`hardware/IMU660RB/imu660rb.c`、`imu660rb.h`

逐飞科技随模块提供的示例驱动，用于把 LSM6DSR 的数据喂给 Fusion。
这部分请遵守逐飞的使用条款（通常允许在逐飞硬件上自由使用）。

---

## 6. 构建工具（不入库）

- **CCS（Code Composer Studio，Theia 版）** — 免费，需自行安装
- **MSPM0 SDK 2.10.00.04** — 见第 1 条
- **XDS-110 调试器** — LP-MSPM0G3507 板载
