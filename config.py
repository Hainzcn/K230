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
# CHN0 直接绑到显示层，不参与算法。CHN1 是算法主输入。
DISPLAY_CHN = 0          # CAM_CHN_ID_0
ALGO_CHN = 1             # CAM_CHN_ID_1
DISPLAY_PIXFORMAT = "YUV420SP"
ALGO_PIXFORMAT = "GRAYSCALE"   # 阶段 A 即采用灰度（plan §4.2 首选）

# 算法分辨率（plan §4.2：320×240 首选；上限 400×240）。
ALGO_WIDTH = 320
ALGO_HEIGHT = 240

# ---------------------------------------------------------------------------
# 算法 ROI（plan §4.3 三段权重子带；阶段 A 仅可视化，不做计算）
# 坐标系：算法分辨率（ALGO_WIDTH × ALGO_HEIGHT）下的 (x, y, w, h)。
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
