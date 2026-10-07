# Sensor_board

配套传感器板配置为 STM32F103R8T6，64 KB Flash、20 KB SRAM，14 路 ADC，SPI1 从机、Mode 0、16 位、MSB first。

控制源码、IOC 来自 `HKUST-Intelligent-Racing/2026_Aerc_line_fol_Ken` 的 `tom` 分支提交 `520b1abfce4c915d34aa6bb53f1e7b0bc77fde91`，保持该版本源文件内容不变。本目录增加了已使用的 CMake、HAL/CMSIS、启动和链接支持以便独立构建。

协议 READ 命令为 `0x00C8`，回复 17 个 16 位字：`0xBEEF`、序号、14 路原始 ADC、异或校验。校验初值为 `0x1357 ^ 0x00C8`，再异或此前 16 个字。与根目录主板版本匹配；其他 UART 或纯 14-word SPI 版本不可直接混用。

构建命令见根 README。原始来源：[Ken 的 tom 分支](https://github.com/HKUST-Intelligent-Racing/2026_Aerc_line_fol_Ken/tree/520b1abfce4c915d34aa6bb53f1e7b0bc77fde91/Sensor_board)。
