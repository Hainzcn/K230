/**
 * @file    ms901m.h
 * @brief   ATK-MS901M 串口姿态传感器流式二进制协议解析（C 移植版）
 *
 *  从 docs/chore/Ms901mStreamParser.{cpp,h} 移植而来，关键差异：
 *    - 去掉 Qt（QByteArray / QList / QString），全部静态缓冲 + 标志位；
 *    - 用 float 替代 double（Cortex-M0+ 无 FPU，float 走 soft-float
 *      但比 double 短 2~3×，配合编译器 single-precision math 链路）；
 *    - 解析改为字节级状态机，免除 mid()/append() 缓冲压缩开销；
 *    - 不输出 19 元素 Snapshot 数组，改为字段化 `ms901m_snapshot_t`，
 *      与 app_telemetry / VOFA 通道映射强绑定；
 *    - 校验和算法保持一致：sum(0x55, 0x55, ID, LEN, DATA[*]) & 0xFF
 *      与 ID = 上一字节的下一字节比对。
 *
 *  帧结构：0x55 0x55 <ID> <LEN> <DATA[LEN]> <CHECKSUM>
 *
 *    ID 0x01: 姿态     LEN=6    roll/pitch/yaw       (int16 LE / 32768 * 180°)
 *    ID 0x02: 四元数   LEN=8    q0/q1/q2/q3          (int16 LE / 32768)
 *    ID 0x03: gyro+acc LEN=12   ax/ay/az/gx/gy/gz    (int16 LE 量纲见 .c)
 *    ID 0x04: mag+temp LEN=8    mx/my/mz / temp(/100)
 *    ID 0x05: baro+alt LEN=10   pressure(int32 Pa) / altitude(int32 / 100 m)
 *
 *  本工程业务上仅使用 0x01（pitch 主用）+ 0x03（gy 用作角速度）+ 0x04（温度），
 *  0x02/0x05 仍解析以备扩展，但不进 snapshot。
 */

#ifndef MS901M_H
#define MS901M_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  最近解析快照（字段化）。仅在主循环线程访问，无并发顾虑。
 *
 *  量纲选择（与 VOFA / 平衡环习惯对齐）：
 *    - 角度：度（°）
 *    - 角速度：度/秒（°/s）—— 直接喂 PD 速率项，无需 rad 转换
 *    - 加速度：g（重力加速度倍数）—— 静态时 ax² + ay² + az² ≈ 1
 *    - 温度：°C
 */
typedef struct {
    float pitch_deg;        /* 0x01 帧：俯仰角 (°)，平衡环主用 */
    float roll_deg;         /* 0x01 帧：横滚角 (°) */
    float yaw_deg;          /* 0x01 帧：偏航角 (°)，磁干扰时不可信 */

    float gx_dps;           /* 0x03 帧：陀螺 X (°/s) */
    float gy_dps;           /* 0x03 帧：陀螺 Y (°/s)，平衡环 pitch 速率 */
    float gz_dps;           /* 0x03 帧：陀螺 Z (°/s) */

    float ax_g;             /* 0x03 帧：加速度 X (g) */
    float ay_g;             /* 0x03 帧：加速度 Y (g) */
    float az_g;             /* 0x03 帧：加速度 Z (g) */

    float temp_c;           /* 0x04 帧：内温度 (°C) */

    bool  has_attitude;     /* 0x01 至少收到过一次 */
    bool  has_gyro_acc;     /* 0x03 至少收到过一次 */
    bool  has_mag_temp;     /* 0x04 至少收到过一次 */
} ms901m_snapshot_t;

/** MS901M 上位机指令帧的固定头 0x55 0xAF。 */
#define MS901M_CMD_SYNC1          0x55u
#define MS901M_CMD_SYNC2          0xAFu

/** 通用指令帧建议缓冲上限：2(sync)+1(id)+1(len)+32(data)+1(sum)=37。 */
#define MS901M_CMD_FRAME_MAX      37u

typedef enum {
    MS901M_CMD_SAVE        = 0x00u,
    MS901M_CMD_SENCAL      = 0x01u,
    MS901M_CMD_SENSTA      = 0x02u,
    MS901M_CMD_GYROFSR     = 0x03u,
    MS901M_CMD_ACCFSR      = 0x04u,
    MS901M_CMD_GYROBW      = 0x05u,
    MS901M_CMD_ACCBW       = 0x06u,
    MS901M_CMD_BAUD        = 0x07u,
    MS901M_CMD_RETURNSET   = 0x08u,
    MS901M_CMD_RETURNSET2  = 0x09u,
    MS901M_CMD_RETURNRATE  = 0x0Au,
    MS901M_CMD_ALG         = 0x0Bu,
    MS901M_CMD_ASM         = 0x0Cu,
    MS901M_CMD_GAUCAL      = 0x0Du,
    MS901M_CMD_BAUCAL      = 0x0Eu,
    MS901M_CMD_LEDOFF      = 0x0Fu,
    MS901M_CMD_D0MODE      = 0x10u,
    MS901M_CMD_D1MODE      = 0x11u,
    MS901M_CMD_D2MODE      = 0x12u,
    MS901M_CMD_D3MODE      = 0x13u,
    MS901M_CMD_D1PULSE     = 0x16u,
    MS901M_CMD_D3PULSE     = 0x1Au,
    MS901M_CMD_D1PERIOD    = 0x1Fu,
    MS901M_CMD_D3PERIOD    = 0x23u,
    MS901M_CMD_RESET       = 0x7Fu
} ms901m_cmd_id_t;

typedef enum {
    MS901M_SENCAL_ACC      = 0x00u,
    MS901M_SENCAL_MAG      = 0x01u,
    MS901M_SENCAL_BARO_ZERO = 0x02u
} ms901m_sencal_t;

typedef enum {
    MS901M_GYRO_FSR_250DPS  = 0x00u,
    MS901M_GYRO_FSR_500DPS  = 0x01u,
    MS901M_GYRO_FSR_1000DPS = 0x02u,
    MS901M_GYRO_FSR_2000DPS = 0x03u
} ms901m_gyro_fsr_t;

typedef enum {
    MS901M_ACC_FSR_2G  = 0x00u,
    MS901M_ACC_FSR_4G  = 0x01u,
    MS901M_ACC_FSR_8G  = 0x02u,
    MS901M_ACC_FSR_16G = 0x03u
} ms901m_acc_fsr_t;

typedef enum {
    MS901M_BAUD_921600 = 0x00u,
    MS901M_BAUD_460800 = 0x01u,
    MS901M_BAUD_256000 = 0x02u,
    MS901M_BAUD_230400 = 0x03u,
    MS901M_BAUD_115200 = 0x04u,
    MS901M_BAUD_57600  = 0x05u,
    MS901M_BAUD_38400  = 0x06u,
    MS901M_BAUD_19200  = 0x07u,
    MS901M_BAUD_9600   = 0x08u,
    MS901M_BAUD_4800   = 0x09u,
    MS901M_BAUD_2400   = 0x0Au
} ms901m_baud_t;

typedef enum {
    MS901M_RETURN_RATE_250HZ = 0x00u,
    MS901M_RETURN_RATE_200HZ = 0x01u,
    MS901M_RETURN_RATE_125HZ = 0x02u,
    MS901M_RETURN_RATE_100HZ = 0x03u,
    MS901M_RETURN_RATE_50HZ  = 0x04u,
    MS901M_RETURN_RATE_20HZ  = 0x05u,
    MS901M_RETURN_RATE_10HZ  = 0x06u,
    MS901M_RETURN_RATE_5HZ   = 0x07u,
    MS901M_RETURN_RATE_2HZ   = 0x08u,
    MS901M_RETURN_RATE_1HZ   = 0x09u
} ms901m_return_rate_t;

typedef enum {
    MS901M_ALG_6_AXIS = 0x00u,
    MS901M_ALG_9_AXIS = 0x01u
} ms901m_alg_t;

typedef enum {
    MS901M_ASM_HORIZONTAL = 0x00u,
    MS901M_ASM_VERTICAL   = 0x01u
} ms901m_asm_t;

typedef enum {
    MS901M_SWITCH_OFF = 0x00u,
    MS901M_SWITCH_ON  = 0x01u
} ms901m_switch_t;

typedef enum {
    MS901M_PORT_MODE_ANALOG_IN   = 0x00u,
    MS901M_PORT_MODE_DIGITAL_IN  = 0x01u,
    MS901M_PORT_MODE_DIGITAL_HI  = 0x02u,
    MS901M_PORT_MODE_DIGITAL_LO  = 0x03u,
    MS901M_PORT_MODE_PWM_OUT     = 0x04u
} ms901m_port_mode_t;

typedef enum {
    MS901M_RETURN_MASK_ATTITUDE  = (1u << 0),
    MS901M_RETURN_MASK_QUAT       = (1u << 1),
    MS901M_RETURN_MASK_GYRO_ACC   = (1u << 2),
    MS901M_RETURN_MASK_MAG        = (1u << 3),
    MS901M_RETURN_MASK_BARO       = (1u << 4),
    MS901M_RETURN_MASK_PORT       = (1u << 5),
    MS901M_RETURN_MASK_ANON       = (1u << 6)
} ms901m_return_mask_t;

/**
 * @brief  初始化 / 复位解析器状态机与所有最新帧标志。
 * @param  acc_fsr_g   加速度计满量程 (g)，与 MS901M 寄存器配置一致；典型 4。
 * @param  gyro_fsr_dps 陀螺仪满量程 (°/s)，典型 2000。
 *
 *  注：ATK 出厂默认 ±4 g / ±2000 dps（与 cpp 版默认一致）。如上位机被改过
 *  量程，需在此同步传入对应数值，否则单位换算系数错位。
 */
void ms901m_init(int16_t acc_fsr_g, int16_t gyro_fsr_dps);

/**
 * @brief  把 UART RX 字节流喂给状态机，按需更新内部最新快照。
 *         典型调用：主循环每 1 ms 调一次，单次 ≤ 64 B。
 */
void ms901m_feed_bytes(const uint8_t *p, size_t n);

/**
 * @brief  构造一帧 MS901M 读寄存器命令：0x55 0xAF (cmd|0x80) 0x01 0x00 sum。
 * @param  cmd_id   目标寄存器 / 指令 ID（原始 7-bit 命令号，不带 bit7）。
 * @param  out      输出缓冲。
 * @param  out_cap  输出缓冲容量，至少 6 字节。
 * @return 帧长度（成功固定为 6），失败返回 0。
 */
size_t ms901m_build_read_cmd(uint8_t cmd_id, uint8_t *out, size_t out_cap);

/**
 * @brief  构造一帧 MS901M 写命令：0x55 0xAF cmd len data... sum。
 * @param  cmd_id     目标寄存器 / 指令 ID。
 * @param  data       数据指针；len=0 时可传 NULL。
 * @param  data_len   数据长度。
 * @param  out        输出缓冲。
 * @param  out_cap    输出缓冲容量，至少 5 + data_len 字节。
 * @return 实际帧长度，失败返回 0。
 */
size_t ms901m_build_write_cmd(uint8_t cmd_id, const uint8_t *data, uint8_t data_len,
    uint8_t *out, size_t out_cap);

/** 便捷封装：构造 1 字节写命令。 */
size_t ms901m_build_write_u8_cmd(uint8_t cmd_id, uint8_t value, uint8_t *out, size_t out_cap);

/** 便捷封装：构造 2 字节 little-endian 写命令。 */
size_t ms901m_build_write_u16_cmd(uint8_t cmd_id, uint16_t value, uint8_t *out, size_t out_cap);

/** 构造保存当前配置到 Flash 的命令。 */
size_t ms901m_build_save_cmd(uint8_t *out, size_t out_cap);

/** 构造恢复默认设置命令。 */
size_t ms901m_build_reset_cmd(uint8_t *out, size_t out_cap);

/** 构造传感器校准命令。 */
size_t ms901m_build_sensor_cal_cmd(ms901m_sencal_t cal, uint8_t *out, size_t out_cap);

/** 构造设置陀螺仪量程命令。 */
size_t ms901m_build_set_gyro_fsr_cmd(ms901m_gyro_fsr_t fsr, uint8_t *out, size_t out_cap);

/** 构造设置加速度计量程命令。 */
size_t ms901m_build_set_acc_fsr_cmd(ms901m_acc_fsr_t fsr, uint8_t *out, size_t out_cap);

/** 构造设置 UART 波特率命令。 */
size_t ms901m_build_set_baud_cmd(ms901m_baud_t baud, uint8_t *out, size_t out_cap);

/** 构造设置主动上报内容命令（bit 定义见 ms901m_return_mask_t）。 */
size_t ms901m_build_set_return_mask_cmd(uint8_t mask, uint8_t *out, size_t out_cap);

/** 构造设置主动上报速率命令。 */
size_t ms901m_build_set_return_rate_cmd(ms901m_return_rate_t rate, uint8_t *out, size_t out_cap);

/** 构造设置姿态解算算法命令。 */
size_t ms901m_build_set_alg_cmd(ms901m_alg_t alg, uint8_t *out, size_t out_cap);

/** 构造设置安装方向命令。 */
size_t ms901m_build_set_asm_cmd(ms901m_asm_t asm_mode, uint8_t *out, size_t out_cap);

/** 构造设置陀螺仪自校准开关命令。 */
size_t ms901m_build_set_gaucal_cmd(ms901m_switch_t enable, uint8_t *out, size_t out_cap);

/** 构造设置气压计自校准开关命令。 */
size_t ms901m_build_set_baucal_cmd(ms901m_switch_t enable, uint8_t *out, size_t out_cap);

/** 构造设置 LED 开关命令。注意寄存器名为 LEDOFF：0=开，1=关。 */
size_t ms901m_build_set_ledoff_cmd(ms901m_switch_t led_off, uint8_t *out, size_t out_cap);

/** 构造设置 D0~D3 端口模式命令。 */
size_t ms901m_build_set_port_mode_cmd(uint8_t port_index, ms901m_port_mode_t mode,
    uint8_t *out, size_t out_cap);

/** 构造设置 D1/D3 PWM 高电平脉宽命令，单位 us。 */
size_t ms901m_build_set_pwm_pulse_cmd(uint8_t port_index, uint16_t pulse_us,
    uint8_t *out, size_t out_cap);

/** 构造设置 D1/D3 PWM 周期命令，单位 us。 */
size_t ms901m_build_set_pwm_period_cmd(uint8_t port_index, uint16_t period_us,
    uint8_t *out, size_t out_cap);

/** 把寄存器编码的加速度计量程选择值同步为本地换算系数。 */
bool ms901m_apply_acc_fsr(ms901m_acc_fsr_t fsr);

/** 把寄存器编码的陀螺仪量程选择值同步为本地换算系数。 */
bool ms901m_apply_gyro_fsr(ms901m_gyro_fsr_t fsr);

/**
 * @brief  返回 0x01 帧是否至少收到过一次（即 pitch 是否就绪）。
 *         主控启动期 ms901m_has_attitude() == false 应视为 IMU 未在线。
 */
bool ms901m_has_attitude(void);

/**
 * @brief  把内部最新快照拷贝给上层（深拷贝，调用方持有副本可自由用）。
 */
void ms901m_get_snapshot(ms901m_snapshot_t *out);

/** 返回累计校验失败 / 长度异常的帧数（1 Hz 自测可监控误码率）。 */
uint32_t ms901m_bad_frames(void);

/** 返回累计成功解析的帧数。 */
uint32_t ms901m_good_frames(void);

#ifdef __cplusplus
}
#endif

#endif /* MS901M_H */
