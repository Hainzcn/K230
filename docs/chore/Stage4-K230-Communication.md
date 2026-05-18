# 阶段 4 ｜ K230 通讯（IMU TX 一分二方案）

> 文档定位：Stage 4 第一步——MCU-K230 通讯链路实现记录。
>
> 关联文档：
> - 项目总览与执行计划：[Overview.md](../Overview/Overview.md)
> - 引脚分配唯一真源：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 帧协议实现：[`template/middle/k230_protocol.{c,h}`](../../template/middle/k230_protocol.h)
> - BSP 驱动层：[`template/hardware/bsp_k230_uart.{c,h}`](../../template/hardware/bsp_k230_uart.h)
> - 集成入口：[`template/app/app_balance.c`](../../template/app/app_balance.c)

---

## 1. 架构决策：IMU TX 一分二

### 1.1 原方案

```
MS901M TX ─→ MCU UART3 RX (PB13)
                    │
                    ↓ (MCU 帧编码)
            MCU UART1 TX (PB6) ──DMA──→ K230 UART RX
```

MCU 需要：
- 在 100 Hz 控制环中构建 `IMU_TELEM` 帧（pitch、pitch_rate、车速、状态）
- 通过 UART1 TX DMA 推送给 K230
- 占用 2 个 DMA 通道（TX + RX）

### 1.2 新方案（Stage 4 采纳）

```
MS901M TX ──┬──→ MCU UART3 RX (PB13)    [已有, 不变]
            └──→ K230 独立 UART RX        [新增 Y 分线]

MCU UART1 TX (PB6) ──阻塞写──→ K230 命令 UART RX    [速度/状态/心跳, ~240 B/s]
MCU UART1 RX (PB7) ←──DMA────  K230 命令 UART TX    [运动指令, DMA 保留]
```

### 1.3 可行性分析

**电气**：
- MS901M TX 是标准 CMOS 推挽输出，驱动两个高阻 UART RX 仅增加门极电容负载
- 115200 baud 位周期 ~8.7 µs，寄生电容影响可忽略
- 三端均为 3.3 V 电平，直连兼容

**协议**：
- MS901M 是单向广播（200 Hz 主动上报 0x01/0x02/0x03 三帧），两端独立解析互不干扰
- K230 直接获得原始 6 轴数据，延迟低于主控转发，云台前馈精度更高

**资源**：
- 释放 UART1 TX DMA 通道（原 DMA_CH1, 硬件通道 0）
- MCU→K230 数据量降至 ~240 B/s，阻塞写无压力

### 1.4 接线方式

| 信号线 | 起点 | 终点 |
|--------|------|------|
| MS901M TX | MS901M 模块 TX 引脚 | Y 分：MCU PB13 (BP J2.26) + K230 UART RX |
| MS901M RX | MCU PB12 (BP J4.32) | MS901M 模块 RX 引脚（配置/校准命令，不分线） |
| K230 CMD TX | K230 UART TX | MCU PB7 (UART1 RX) |
| K230 CMD RX | MCU PB6 (UART1 TX) | K230 UART RX (命令通道) |
| GND | 全部共地 | 单点星型汇接 |

---

## 2. 帧协议设计

### 2.1 帧格式

```
字节偏移   内容              说明
0          0xAA              帧头高字节
1          0x55              帧头低字节
2          LEN               PAYLOAD 字节数 (0~32)
3          CMD               帧类型标识
4..4+LEN-1 PAYLOAD[LEN]      业务数据
4+LEN      CRC16_LO          CRC16-CCITT 低 8 位
5+LEN      CRC16_HI          CRC16-CCITT 高 8 位
6+LEN      0x55              帧尾低字节
7+LEN      0xAA              帧尾高字节
```

- CRC16-CCITT：多项式 0x1021，初始值 0xFFFF，校验范围 = `LEN + CMD + PAYLOAD`
- 帧尾用于二次确认帧边界

### 2.2 MCU → K230 帧

| CMD | 名称 | 频率 | PAYLOAD 布局 | 大小 |
|-----|------|------|-------------|------|
| 0x01 | VEHICLE_STATUS | 20 Hz | `avg_cps:i32 + safety_state:u8 + bat_mv:u16` | 7 B |
| 0x02 | HEARTBEAT_MCU | 1 Hz | `uptime_ms:u32` | 4 B |

### 2.3 K230 → MCU 帧

| CMD | 名称 | 频率 | PAYLOAD 布局 | 大小 |
|-----|------|------|-------------|------|
| 0x11 | MOTION_CMD | 20~50 Hz | `target_v:i16 + target_omega:i16 + mode:u8` | 5 B |
| 0x12 | HEARTBEAT_K230 | 1 Hz | `uptime_ms:u32` | 4 B |
| 0x13 | PID_INJECT | 按需 | `pid_id:u8 + kp:f32 + ki:f32 + kd:f32` | 13 B |

### 2.4 带宽估算

MCU→K230（20 Hz × 15 B + 1 Hz × 12 B = 312 B/s）：
- 921600 baud 有效吞吐 ~92 kB/s，占用率 < 0.4%
- 阻塞写每字节 ~10.9 µs，15 B 帧 ~164 µs，对 50 ms 速度环周期占比 0.3%

K230→MCU（50 Hz × 13 B + 1 Hz × 12 B = 662 B/s）：
- DMA 接收，不占 CPU

---

## 3. SysConfig 与生成文件变更

### 3.1 LP_MSPM0G3507.syscfg

移除 K230 UART 的 TX DMA 配置：
```diff
- K230_UART.enabledDMATXTriggers = "DL_UART_DMA_INTERRUPT_TX";
- K230_UART.enableDMATX          = true;
```

### 3.2 ti_msp_dl_config.h

- 移除 `DMA_CH1` 相关定义（TX DMA 通道）
- 新增 `DMA_CH_UART_K230_DMA_RX_CHAN` 映射宏，解决之前 fallback 默认值 0 与实际通道 1 不一致的 bug

### 3.3 ti_msp_dl_config.c

- 移除 `DL_UART_Main_enableDMATransmitEvent` 调用
- 移除 `gDMA_CH1Config` 和 `SYSCFG_DL_DMA_CH1_init`
- 修正 `gDMA_CH0Config`：改为 `DL_DMA_WIDTH_BYTE` + `DL_DMA_ADDR_INCREMENT`（dest），匹配 UART RX 字节传输语义

---

## 4. 软件实现

### 4.1 k230_protocol.c/.h（新增）

- CRC16-CCITT 查表法（256 B ROM）
- `k230_encode_frame`：帧编码（head + len + cmd + payload + crc + tail）
- `k230_parser_t` + `k230_parser_feed`：逐字节状态机解析器
- 所有 payload 结构体 `__attribute__((packed))` 保证 MCU/K230 间二进制兼容

### 4.2 bsp_k230_uart.c（重构）

- RX：DMA BLOCK 模式（256 B 缓冲传满中断 → 搬入 512 B 应用环缓 → 重装）
- TX：保留 `bsp_k230_uart_write_blocking`，TX DMA 相关代码全部移除
- 新增 `bsp_k230_uart_rx_pop_bulk` 供上层批量读取
- 修正 DMA 通道号：使用 `DMA_CH_UART_K230_DMA_RX_CHAN`（= DMA_CH0_CHAN_ID = 1）

### 4.3 app_balance.c（集成）

主循环中新增 K230 通讯调度：

| 频率 | 操作 |
|------|------|
| 1 kHz | `k230_drain_and_dispatch`：从环缓取字节 → 喂 parser → 分发已完成帧 |
| 1 kHz | `k230_check_timeout`：500 ms 无帧 → 运动指令归零 |
| 20 Hz | `k230_send_vehicle_status`：编码 + 阻塞写 VEHICLE_STATUS |
| 1 Hz | `k230_send_heartbeat`：编码 + 阻塞写 HEARTBEAT_MCU |

帧分发逻辑：
- `MOTION_CMD` (0x11)：解包后直接覆盖 `cmd.target_speed_cps` / `cmd.target_yaw_pm`
- `PID_INJECT` (0x13)：按 `pid_id` 调用对应 `set_*_gains` API
- `HEARTBEAT_K230` (0x12)：仅刷新最后收帧时间戳

1 Hz 心跳日志新增字段：`k230_g=<good>/b=<bad> k230_<ON|OFF>`

---

## 5. K230 侧准备事项

K230 端不在本工程范围内，但以下是对接所需：

1. **IMU 解析**：移植 `ms901m.c/.h` 到 K230（C 或 MicroPython），接收 115200 UART 数据
2. **命令帧编码**：实现 `0xAA 0x55` 帧格式编码（发送 MOTION_CMD / HEARTBEAT_K230 / PID_INJECT）
3. **状态帧解码**：实现帧解析状态机（接收 VEHICLE_STATUS / HEARTBEAT_MCU）
4. **硬件接线**：K230 需要两路 UART——一路 115200 接 MS901M TX Y 分线，一路 921600 接 MCU UART1

---

## 6. K230 联调日志判读（2026-05-18）

联调期 K230 侧曾临时增加 MCU 上行坏帧分类日志，格式如下：

```text
mcu_bad=lenX/t1Y/t2Z/crcW last=reason:Lxx:Cyy:rx/calc
```

这些字段已在串口稳定后从常规 5 s 日志中移除，底层计数器仍保留，后续排障可重新打开。历史字段含义：

| 字段 | 含义 |
|---|---|
| `lenX` | `LEN > 32` 的坏帧数 |
| `t1Y` | 帧尾第 1 字节不是 `0x55` 的坏帧数 |
| `t2Z` | 帧尾第 2 字节不是 `0xAA` 的坏帧数 |
| `crcW` | CRC16 校验不匹配的坏帧数 |
| `last=crc:L4:C02:rx/calc` | 最近坏帧为 CRC 错；`LEN=4`、`CMD=0x02`、收到 CRC=`rx`、K230 本地计算 CRC=`calc` |

### 6.1 当前实测现象

K230 运行日志节选：

```text
mcu=OFF bat=10909mV cps=+0 mcu_g=88/b=328  uart_rx=6180B
mcu_bad=len0/t10/t20/crc328 last=crc:L4:C02:970B/8A75

mcu=ON  bat=10909mV cps=+0 mcu_g=192/b=330 uart_rx=7755B
mcu_bad=len0/t10/t20/crc330 last=crc:L4:C02:0FF8/0F58

mcu=OFF bat=10909mV cps=+0 mcu_g=498/b=340 uart_rx=12450B
mcu_bad=len0/t10/t20/crc340 last=crc:L4:C02:E329/E309
```

判读：

1. `uart_rx` 持续增长，且速率接近 `312 B/s`，说明 UART2 物理链路、波特率、接线方向基本正确。
2. `len0/t10/t20` 全为 0，说明帧头、`LEN`、帧尾边界基本正确，没有明显多发/少发字节。
3. `bat=10909mV` 且 `mcu_g` 持续增长，说明 `VEHICLE_STATUS (CMD=0x01, LEN=7)` 已经能被 K230 正确解析。
4. `last=crc:L4:C02` 指向 `HEARTBEAT_MCU (CMD=0x02, LEN=4)`，说明当前主要问题集中在 **MCU→K230 心跳帧 CRC 不一致**。
5. K230 侧 `mcu=ON/OFF` 抖动的直接原因是旧版 K230 只在收到合法 `HEARTBEAT_MCU` 后刷新在线时间戳；状态帧即使正常，也不会刷新 heartbeat 时间戳。

### 6.2 MCU 侧优先核对项

重点检查 `k230_send_heartbeat` 或等效心跳发送路径：

1. `LEN` 必须为 `4`，只表示 payload 长度，不包含 `CMD`、CRC、帧头、帧尾。
2. `CMD` 必须为 `0x02`。
3. payload 必须为 `uptime_ms:u32`，小端序。
4. CRC16-CCITT 参数必须与 K230 一致：
   - poly = `0x1021`
   - init = `0xFFFF`
   - xorout = `0x0000`
   - 不反射输入/输出
5. CRC 校验范围必须是：

```text
LEN + CMD + PAYLOAD
```

即心跳帧为：

```text
04 02 <uptime_ms little-endian 4B>
```

不包含 `AA 55` 帧头，也不包含 `55 AA` 帧尾。

6. CRC 字节序必须低字节在前：

```text
CRC16_LO CRC16_HI
```

完整心跳帧应为：

```text
AA 55 04 02 <uptime_ms[0..3]> <crc_lo> <crc_hi> 55 AA
```

### 6.3 对接建议

为避免 K230 侧在线状态被单一心跳 CRC 问题拖累，已按 MCU 侧“无任何帧超时”的语义调整 K230 在线判定：

1. **K230 侧修复**：收到任意合法 MCU 上行帧（`VEHICLE_STATUS` 或 `HEARTBEAT_MCU`）都刷新在线时间戳；这样状态帧正常时不会 `MCU:ON/OFF` 抖动。
2. **MCU 侧仍需修复**：`HEARTBEAT_MCU` 的 CRC 不一致仍会累积 `mcu_bad=crc`，应继续核对 §6.2 的心跳 CRC 范围/字节序。修复后 `mcu_bad=crc` 应停止增长。

---

## 7. CRC 查找表错误根因分析与修复（2026-05-18）

### 7.1 根因

K230 日志确认：`VEHICLE_STATUS (0x01)` 帧 CRC 正常；`HEARTBEAT_MCU (0x02)` 帧 CRC 持续不一致。

对 `k230_protocol.c` 中的 CRC16-CCITT 查找表进行全量验证（Python 生成正确表后逐条对比），发现**原表共有 50 处错误**，分布在以下 index 区间：

| 索引范围 | 错误数 | 典型差值（低字节） |
|---------|------|----------------|
| 6, 7 | 2 | `0x50`（bits 4+6 错） |
| 39~47 | 10 | `0x20`（bit 5 错） |
| 64~95 | 32 | `0xA0` 或 `0x80`（bits 5+7 错） |
| 212~223 | 6 | `0x20`（bit 5 错） |

这些错误是表初始生成时的数值转录问题，并非算法逻辑错误。

### 7.2 为何 VEHICLE_STATUS 未受影响

CRC 计算中命中哪些表项取决于中间 CRC 值与当前数据字节的 XOR 索引。`VEHICLE_STATUS` 的特定 payload（avg_cps 多为 0、bat_mv≈10900）恰好未命中上述 50 个错误条目；`HEARTBEAT_MCU` 的 uptime_ms 随时间递增，每秒都会命中不同表项，因此几乎每帧必然出错。

### 7.3 修复方案

将查找表及其 `k230_crc16` 函数整体替换为**按位计算实现**：

```c
uint16_t k230_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i; int b;
    for (i = 0u; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc<<1u)^0x1021u)
                                  : (uint16_t)(crc<<1u);
        }
    }
    return crc;
}
```

- 与 K230 侧 Python 实现完全等价（已用实测日志反推验证：uptime_ms=4000 ms → CRC=0x8A75，与 K230 日志中 `calc` 字段吻合）。
- 本工程 MCU→K230 吞吐量 < 1 kB/s，按位计算对 80 MHz M0+ 的 CPU 占用远低于 0.1%，无性能顾虑。

### 7.4 预期效果

修复后重新烧录：
- `mcu_bad=crc` 应**停止增长**（仍可能有极少量因链路干扰导致的偶发坏帧）
- `mcu=ON/OFF` 抖动消失，稳定显示 `mcu=ON`

---

## 8. 验证清单

- [ ] SysConfig 重新生成后编译通过（或手工修改 ti_msp_dl_config 已等效）
- [ ] 上电后 1 Hz 心跳日志出现 `k230_g=0/b=0 k230_OFF`（K230 未连接时预期行为）
- [ ] PB6 用 USB-TTL 监听可见 20 Hz VEHICLE_STATUS + 1 Hz HEARTBEAT_MCU 帧
- [ ] PB7 用 USB-TTL 发送构造的 MOTION_CMD 帧 → 车辆响应 v/ω 指令
- [ ] 发送后断开 → 500 ms 后自动归零 + 日志显示 `k230_OFF`
- [ ] PB6/PB7 回环短接 → 帧计数递增（验证编解码一致性）
- [ ] MS901M TX Y 分线后两端同时收到 200 Hz 数据（万用表 / 逻辑分析仪验证）

---

## 9. 变更日志

| 日期 | 版本 | 内容 | 执行方 |
|------|------|------|--------|
| 2026-05-17 | v0.1 | Stage 4 第一步：IMU TX 一分二方案决策 + MCU 侧帧协议全量实现 | 主控团队 |
| 2026-05-18 | v0.2 | 补充 K230 实测日志判读：状态帧已通，心跳帧 `CMD=0x02/LEN=4` 出现 CRC 不一致，导致 K230 `MCU:ON/OFF` 抖动 | K230 联调 |
| 2026-05-18 | v0.3 | K230 在线判定改为任意合法 MCU 上行帧刷新，与 MCU 侧“无任何帧超时”语义对齐；心跳 CRC 问题保留 bad 统计继续追踪 | K230 联调 |
| 2026-05-18 | v0.4 | 发现 CRC16-CCITT 查找表共 50 处转录错误；移除错误表改用按位计算实现（k230_protocol.c），修复后两端 CRC 对齐，mcu_bad=crc 停止增长 | 主控团队 |
| 2026-05-18 | v0.5 | K230 常规运行日志移除 `mcu_bad/uart_rx/avg_us` 等临时诊断字段，仅保留基础链路状态；详细诊断保留为历史排障记录 | K230 联调 |
