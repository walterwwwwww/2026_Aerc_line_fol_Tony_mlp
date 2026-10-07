# 来源与许可

- 主板控制代码基础：`Tony12290427/linfol_final` 的 `5-6-2025`，提交 `b50b018d43dddd55d697eda4eaa605629a5cd97f`；本分支是在此基础上的已验证修改。
- 传感器板：`HKUST-Intelligent-Racing/2026_Aerc_line_fol_Ken`，`tom`，提交 `520b1abfce4c915d34aa6bb53f1e7b0bc77fde91`，`Sensor_board/Core/` 和 IOC 原样保留。
- F4 HAL 版本 1.8.5，保留本次构建所用源文件和头文件；官方许可来自 [ST v1.8.5 LICENSE.md](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/v1.8.5/LICENSE.md)，位于 `Vendor/STM32F4/STM32F4xx_HAL_Driver/LICENSE.md`。
- F1 HAL 版本 1.1.10，位于 `Sensor_board/Drivers/STM32F1xx_HAL_Driver/`，附原许可。
- ARM CMSIS Core 头文件与两系列 STM32 CMSIS Device 头文件保留对应许可及源文件版权头。启动文件也保留原版权说明；F1 启动附 `bundled-startup-LICENSE.md`。

仅打包所需 MCU 头文件、HAL 编译模块和通用头文件，未更改供应商源文件内容。驱动版本按本次实际构建记录，未使用自动下载最新版替代。上述许可分别适用于对应第三方文件，不代表给整个应用另行授予新许可。
