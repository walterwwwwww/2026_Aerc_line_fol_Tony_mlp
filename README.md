# 2026-10-07 demo：主板、传感器板与 PID 上位机

本分支以 `5-6-2025` 的 `b50b018d43dddd55d697eda4eaa605629a5cd97f` 为基础，整理了本次在 STM32F407VET6 主板与 STM32F103R8T6 传感器板上使用的代码和串口调参工具。

## 目录

| 路径 | 内容 |
|---|---|
| `Core/`、`linfol_final.ioc` | F407 主板控制源码与 CubeMX 配置 |
| `CMakeLists.txt`、`Vendor/STM32F4/` | 主板构建入口、匹配的 HAL/CMSIS、启动与链接支持 |
| `Sensor_board/` | 配套 F103 传感器板源码、驱动和独立 CMake 工程 |
| `pid-tuner/` | 本地网页上位机、串口协议解析、测试和 demo 参数草稿 |
| `Tests/` | 传感器权重等价性、独立开关和转弯状态机测试 |
| `docs/` | 来源、验证范围与保留的旧移植说明 |

## 主板当前行为

- 默认基础巡航速度 `base_speed=40`。
- 独立运行时开关 `turn_enable=0/1`：复位默认关闭，关闭保留整张转向表并取消当前强制转向，使用正常 PID 循迹。开启仍仅查前 5 槽，需先停车。
- 阻塞 SPI 采集在主循环完成，1 ms 控制中断使用完整数据快照，避免挤占命令处理和系统计时。
- 停车分支将四个电机 PWM 比较寄存器归零。
- 14 路独立权重可通过 `weight1`–`weight14` 修改，范围为非负幅值 0–100。左右方向由原扫描算法决定。
- 默认权重从 S01 到 S14 为 `6.5, 5, 4, 3, 2, 1, 0.1, 0.1, 1, 2, 3, 4, 5, 6.5`；镜像默认值保留旧计算表达式，外侧优先扫描不变。

`pid-tuner/profiles/2026-10-07-demo.json` 保存了最近已确认的调参值。它是可载入草稿，不会自动发送，也不等同于固件开机默认值。串口改动保存在 RAM，车辆复位后恢复源码默认值，传感器标定也需要重新进行。

当前独立开关版本已编译并通过软件测试；此前 SWD 中断的烧录现已恢复完成，OpenOCD 校验通过，55984 字节回读与镜像逐字节一致。车端开关默认关闭、车辆停止、四路 PWM 比较寄存器为零；参数与烧录前备份一致，基础速度 40。复位后的传感器标定需重新进行。本轮无线串口开关操作尚未实车验证；备份和调试日志保留本地，不随源码提交。

## 构建两块板

需要 CMake 3.22 以上、Ninja，以及带 Newlib/nano.specs 的完整 Arm GNU 工具链。工具链在 PATH 中时可省略 `FIRMWARE_TOOLCHAIN_BIN`；否则指定其 `bin` 目录：

```sh
cmake -S . -B build/main -G Ninja -DCMAKE_BUILD_TYPE=Debug -DFIRMWARE_TOOLCHAIN_BIN=/path/to/arm-gnu/bin
cmake --build build/main
cmake -S Sensor_board -B build/sensor -G Ninja -DCMAKE_BUILD_TYPE=Debug -DFIRMWARE_TOOLCHAIN_BIN=/path/to/arm-gnu/bin
cmake --build build/sensor
```

输出分别为 `build/main/linfol_final.elf/.hex/.bin` 和 `build/sensor/sensor_board.elf/.hex/.bin`。主板链接布局按实物采用 512 KB Flash、128 KB SRAM；传感器板采用 64 KB Flash、20 KB SRAM。构建文件不依赖个人目录路径，也不需要先运行 CubeMX 生成代码。

保留的 IOC 写着 F407VGT6（1 MB），实际验证的板子是 F407VET6（512 KB）；请以本分支链接文件和实物型号为准。不要未经核对用 IOC 重新生成并覆盖已集成的控制代码。

## 上位机

```sh
uv run --script pid-tuner/app.py --open
```

或使用 Python 3.10 以上：

```sh
python -m venv .venv
# 激活 .venv 后执行：
python -m pip install -r pid-tuner/requirements.txt
python pid-tuner/app.py --open
```

Mac 可双击 `pid-tuner/启动上位机.command`。界面为 <http://127.0.0.1:8765/>，选择逐飞无线 USB 接收器对应端口，115200、8N1，点击读取参数/权重。只有车端回复后才显示确认。支持连续录制 ZIP、20 行转向表编辑和独立开关。完整用法见 [上位机说明](pid-tuner/README.md)。

## 测试

```sh
python -m unittest discover -s pid-tuner -p 'test_*.py' -v
mkdir -p build
cc -O0 -I Core/Inc Tests/test_sensor_weights.c -o build/test_sensor_weights
./build/test_sensor_weights
clang -std=c11 -fsanitize=address,undefined -I Tests/adc_mock -I Core/Inc Tests/test_turn_switch.c -lm -o build/test_turn_switch
./build/test_turn_switch
```

串口测试使用 POSIX 伪终端，适合 Mac/Linux；Windows 可使用上位机，但这些伪终端测试未在 Windows 验证。已完成的构建、烧录、回读与串口验证范围见 [验证记录](docs/VALIDATION.md)。

传感器板来源和第三方许可见 [来源说明](docs/SOURCES.md)。旧移植说明保留在 [旧版文档](docs/LEGACY_PORT_GUIDE.md)，其中部分型号、通道数量与集成步骤已不适用于当前 demo。
