"""K230 视觉循迹子系统配置。

阶段 A 阶段：仅包含摄像头采集、显示与调试叠加所需的最小配置项；
后续阶段（光度、IPM、控制律、UART）会按 vision_line_tracking_plan_v2.md
§11 的目录结构持续扩展，本文件保持唯一配置源（single source of truth）。

约定：
- 所有物理量变量后缀带单位（如 ``*_mm``、``*_ms``、``*_px``）。
- 任何阈值/增益必须先在此处声明，禁止脚本内硬编码（plan §11.2）。
- 标定结果统一存放在 ``calib.json``，由 :func:`load_calibration` 在启动期载入。
"""

CONFIG_VERSION = "phaseA-0.1"

# ---------------------------------------------------------------------------
# 显示设备
# ---------------------------------------------------------------------------
# CanMV K230D Zero 默认 ST7701 800×480 LCD（与 camera_single_bind_lcd.py 一致）。
DISPLAY_TYPE = "ST7701"
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TO_IDE = True

# ---------------------------------------------------------------------------
# Sensor 通道
# ---------------------------------------------------------------------------
# CHN0 → display (bind to LCD VIDEO1 layer，零拷贝)；CHN1 → algorithm。
# 阶段 A 实测：snapshot 路径在 CHN0 / CHN1 上都是 ~30 FPS（详见 phase_A.md
# §4.2），swap 没收益，因此回到 plan §4.2 的默认分工。
DISPLAY_CHN = 0
ALGO_CHN = 1
DISPLAY_PIXFORMAT = "YUV420SP"
# GRAYSCALE 与 YUV420SP 实测耗时一致，但 GRAYSCALE 直接喂 cv_lite.grayscale_*
# 系列接口最干净（plan §4.2 首选项），阶段 B 起 line_detector 直接用。
ALGO_PIXFORMAT = "GRAYSCALE"

# 算法分辨率（plan §4.2：320×240 首选；上限 400×240）。
ALGO_WIDTH = 320
ALGO_HEIGHT = 240

# Sensor 原生采集模式（必须三件套一起指定，只传 fps 不生效）。
# 驱动日志形如 ``find sensor ov5647_csi2, output WxH@FPS``；如果日志显示的
# W×H 不等于这里的 REQ 值，说明驱动没匹配到，自动回落到默认 1920×1080，而
# 该默认模式在 OV5647 上最高只能 30 FPS。
#
# OV5647 支持的 (w, h, fps)：
#   2592×1944@10 / 1920×1080@30 / 1280×960@45 / 1280×720@60 / 640×480@90
# GC2093 支持的 (w, h, fps)：
#   1920×1080@30/60 / 1280×960@60 / 1280×720@90
#
# 目标 60 FPS：在 OV5647 上取 1280×720@60 是最小可行组合（>800 能覆盖
# CHN0 800×480 下采样，320×240 CHN1 也够）。要到 90 FPS 必须把 CHN0
# 降到 640×480 以下；暂不做。
SENSOR_REQ_WIDTH = 1280
SENSOR_REQ_HEIGHT = 720
SENSOR_NOMINAL_FPS = 60

# ---------------------------------------------------------------------------
# 算法 ROI（plan §4.3 三段权重子带；阶段 A 仅可视化，不做计算）
# 坐标系：算法分辨率（ALGO_WIDTH × ALGO_HEIGHT）下的 (x, y, w, h)。
#
# 几何说明（plan §4.3）：
#   - 像素等高（各 50 px）是 plan 明文规定，不是简化。
#   - 在 IPM"近密远疏"映射下，像素等高意味着 NEAR 获得最高的 px/mm 密度
#     （NEAR 50 px ≈ 130 mm 物理范围，FAR 50 px ≈ 200 mm）。
#     NEAR 在 e_y 精度上天然占优；fusion 权重 0.5/0.3/0.2（见后续阶段）
#     再把 NEAR 的贡献放到 MID 的约 1.7 倍、FAR 的 2.5 倍。
#   - ROI 底部 y=230（而非 y=240）是刻意留白 10 px，给车体底盘 / 车轮 /
#     阴影的自拍投影做预留掩膜（plan §4.3）。
#     如装配后发现底盘投影实际侵入 y<230 或从未达到 y=230，阶段 B 起再调整。
# ---------------------------------------------------------------------------
ROI_NEAR_PX = (0, 180, 320, 50)
ROI_MID_PX = (0, 130, 320, 50)
ROI_FAR_PX = (0, 80, 320, 50)
# 总 ROI 框（用于在显示叠加层画一个外框，便于硬件镜头瞄准）。
ROI_TOTAL_PX = (0, 80, 320, 150)

# ---------------------------------------------------------------------------
# 调试叠加
# ---------------------------------------------------------------------------
DEBUG_DISPLAY = True
OSD_REFRESH_INTERVAL_MS = 1000     # plan §9.2 守则 7：OSD 不每帧刷新
OSD_TEXT_SIZE_PX = 22
OSD_TEXT_COLOR = (255, 255, 255)
OSD_ALERT_COLOR = (255, 0, 0)      # 丢帧率 / 内存告警时用纯红，高可读性
OSD_ROI_COLOR = (255, 255, 0)
OSD_ROI_THICKNESS = 2

# ---------------------------------------------------------------------------
# 采样模式（用于 plan §12 阶段 A 的"静态赛道样张 ≥ 100 张"任务）
# ---------------------------------------------------------------------------
CAPTURE_ENABLE = False
CAPTURE_DIR = "/sdcard/captures"
CAPTURE_INTERVAL_FRAMES = 30        # 每 N 帧保存一张
CAPTURE_MAX_SAMPLES = 200
CAPTURE_JPEG_QUALITY = 85

# ---------------------------------------------------------------------------
# 资源与性能
# ---------------------------------------------------------------------------
GC_THRESHOLD_BYTES = 64 * 1024
LOG_INTERVAL_MS = 5000               # 控制台日志节流（plan §9.2 守则 9）
SNAPSHOT_TIMEOUT_MS = 200

# OSD 内存告警阈值：仅当发生以下任一情形时，才把内存信息叠加到调试 OSD。
# 正常运行期 OSD 只显示帧率与丢帧率，保持简洁。
MEM_DRIFT_ALERT_PCT = 5.0            # 漂移超过 5% 提示（阶段 A 硬指标是 ≤10%）
MEM_LOW_ALERT_BYTES = 4 * 1024 * 1024  # 剩余内存低于 4 MB 提示

# 帧周期告警阈值（ms）：1000 / algo_fps > 该值时 OSD FPS 行标红。
# 50 ms ≈ 20 FPS，对应 plan §12 阶段 A "带 OSD ≥ 20 FPS" 的底线。
# 注意：不把"期望 sensor 出帧率"作为分母算"drop"——Display.fps() 返回
# 的是 LCD VSync，与 sensor 出帧率不是同一量，拿去做 drop 会永远误报。
FRAME_PERIOD_ALERT_MS = 50.0

# 启动期 raw snapshot 计时探针：>0 时连续阻塞拉 N 帧，统计单次 snapshot
# 的耗时分布并打印；用于把"snapshot 自身阻塞"和"主循环其他开销"分开看。
# 平时设 0 关闭；性能回归时再临时打开。建议 30~100。
PROBE_SNAPSHOT_FRAMES = 0

# ---------------------------------------------------------------------------
# 性能定论（阶段 A 2026-05-04 实测，已穷尽变量）
# ---------------------------------------------------------------------------
# K230 CanMV ``sensor.snapshot()`` 路径有 **~30 FPS 实际上限**，与以下变量
# 都无关（4 维 × 2 状态全部交叉测过）：
#   - 通道：CHN0 vs CHN1 → 30 FPS
#   - 像素格式：GRAYSCALE vs YUV420SP → 30 FPS
#   - IDE 回传：DISPLAY_TO_IDE True/False → 30 FPS
#   - sensor mode：1920×1080@30 → 30 FPS；1280×720@60 → 30 FPS
#
# Probe 数据：min=16 ms (一个 sensor 周期), p50=30 ms (两个 sensor 周期),
# avg=29.8 ms。说明 sensor 物理上能 60 FPS 出帧（min=16 是证据），但 SDK
# 内部 snapshot 流程（VB 归还/防重复返回/MMZ 同步）每次平均开销 ~13.5 ms，
# 拉高总周期到 30 ms。这是 SDK 层硬约束，软件无法突破。
#
# 影响：
#   - plan §12 阶段 A 的"无 OSD ≥ 35 FPS"指标在该 SDK 上不可达，已修订
#     为"≥ 28 FPS"（30 × 95%）。详见 docs/task_log/phase_A.md §4.2。
#   - 后续阶段的算法处理预算 = 30 ms − snapshot 开销，仍有余地，详见
#     plan §9.1 修订。
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 标定文件（阶段 A 不强制存在）
# ---------------------------------------------------------------------------
CALIB_PATH = "/sdcard/calib.json"


def get(key, default=None):
    """安全读取本模块的属性，避免 ``AttributeError`` 中断主循环。"""
    return globals().get(key, default)


def assert_version(required):
    """供后续模块校验配置兼容性。阶段 A 暂不严格匹配，仅打印警告。"""
    if required != CONFIG_VERSION:
        print(
            "[config] WARNING: required=%s but CONFIG_VERSION=%s"
            % (required, CONFIG_VERSION)
        )


def load_calibration(path=None):
    """启动期加载 ``calib.json``。阶段 A 仅占位，返回空 dict。

    阶段 C 完成 IPM 标定后再实现完整的 JSON 解析与字段校验。
    """
    return {}
