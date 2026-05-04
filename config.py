"""K230 视觉循迹子系统配置。

阶段 A 阶段：仅包含摄像头采集、显示与调试叠加所需的最小配置项；
后续阶段（光度、IPM、控制律、UART）会按 vision_line_tracking_plan_v2.md
§11 的目录结构持续扩展，本文件保持唯一配置源（single source of truth）。

约定：
- 所有物理量变量后缀带单位（如 ``*_mm``、``*_ms``、``*_px``）。
- 任何阈值/增益必须先在此处声明，禁止脚本内硬编码（plan §11.2）。
- 标定结果统一存放在 ``calib.json``，由 :func:`load_calibration` 在启动期载入。
"""

CONFIG_VERSION = "phaseB-0.1"

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
# 阶段 B：光度自适应（plan §5.3 + §6.6）
# ---------------------------------------------------------------------------
# 启动 bootstrap 前的保底阈值。一旦 photometric.bootstrap() 完成就被覆盖。
# 80 是 plan §15 MVP 的硬编码值，实测光照下大多数场景偏暗，作为冷启动可用。
LINE_THRESHOLD_INIT = 80

# bootstrap 采样帧数（plan §5.3 步骤 1：30 帧 ROI）。
PHOTO_BOOTSTRAP_FRAMES = 30

# 运行期漂移检测：每 INTERVAL_MS 取一次 ROI 直方图，若 |Δμ_bg| 超过 TRIG_DELTA_MU
# 就触发一次 30 帧重标定（plan §5.3 运行期策略）。
PHOTO_DRIFT_CHECK_INTERVAL_MS = 1000
PHOTO_DRIFT_TRIG_DELTA_MU = 15.0

# Otsu 直方图分桶；标准 8 位灰度 = 256，不轻易改。
PHOTO_HIST_BINS = 256

# 阈值保底公式 thr_fallback = μ_bg − k·σ_bg（plan §5.3）。3σ 是高斯尾部 0.13%
# 误判，对单色赛道足够。最终 line_threshold = (thr_otsu + thr_fallback) / 2。
#
# 边界：当 σ ≳ μ/k（典型于桌面杂物 / 远景非高斯背景），thr_fallback 会落到 ≤0；
# 此时算法降级为 "只信 Otsu"，同时打印一行诊断日志。详见
# vision/photometric._finalize_recalib。
PHOTO_FALLBACK_K_SIGMA = 3.0

# ---------------------------------------------------------------------------
# 阶段 B：阈值收尾（人工偏置 + 上下限）
# ---------------------------------------------------------------------------
# Plan §5.3 公式给出的是"理想电工胶带场景"的中位估计；实际器材种类（哑光
# 黑塑料、电工胶带、烤漆金属、PVC）反射率差异很大，落到 grayscale 上
# 30~80 都可能。下面三个旋钮在不修算法的前提下做粗调：
#
# - LINE_THRESHOLD_BIAS：加到合成阈值上的偏置；正值 ⇒ "近黑"容差放大，更多
#   灰度像素会被判作前景；负值 ⇒ 收紧。电工胶带（grayscale ~50~80）建议先
#   保留 0；若用更亮的"近黑"材料（如 PVC、磨砂塑料）出现"前景偏稀"，可调
#   到 +10~+20 再看二值图叠加。
LINE_THRESHOLD_BIAS = 0
# - LINE_THRESHOLD_MIN / MAX：硬下限 / 上限。Otsu 在病态场景偶尔会给出极低
#   或极高值（例如全场都是黑色阴影时给 < 20），下限保证黑色目标不会因此完全
#   不被检出；上限避免过度饱和把背景一起判作前景。
LINE_THRESHOLD_MIN = 10
LINE_THRESHOLD_MAX = 60

# ---------------------------------------------------------------------------
# 阶段 B：扫描带几何（plan §6.2）
# ---------------------------------------------------------------------------
# 5 条带均匀铺在 ROI_TOTAL_PX (y=80~230，150 px 高)：
#   y_top[i] = 80 + round((150 - 8) / 4 * i) = 80, 116, 151, 187, 222
# 最后一条 y_top=222、y_bot=230 恰好压到 ROI 底（plan §4.3 给底盘投影预留的 10 px
# 留白由 ROI 自身保证，不再二次缩进）。
BAND_COUNT = 5
BAND_HEIGHT_PX = 8
BAND_TOPS_PX = (80, 116, 151, 187, 222)

# ---------------------------------------------------------------------------
# 阶段 B：L2 硬约束（plan §6.2）
# ---------------------------------------------------------------------------
# mass_i ≥ MIN_MASS_PER_BAND[i]：每带最小累积像素强度。bin_inv 黑线区像素 = 255，
# 所以"完整一条 18 mm 黑线穿过 8 行带 + 在 IPM 近带占 ~20 px 宽"≈
#   8 (rows) × 20 (cols) × 255 = 40800。设 MIN 为该名义值的 25%（10000 量级）；
# 远带因 IPM 透视使黑线变窄，下调到 5000 量级。
# 5 条带顺序：y_top 升序 = NEAR(180~230 区段) → FAR(80~130 区段)，索引 i 越大越近。
MIN_MASS_PER_BAND = (4000, 5000, 6500, 8000, 10000)

# 等效宽度的下界/上界 (px)：col_sum > COL_SUM_THR_FOR_WIDTH 的列数。
# IPM 后近处黑线 ~20 px、远处 ~10 px；±50% 留容差。
W_MIN_PX_PER_BAND = (5, 6, 8, 10, 12)
W_MAX_PX_PER_BAND = (16, 18, 22, 26, 30)
COL_SUM_THR_FOR_WIDTH = 255          # bin_inv 中"该列有黑线像素覆盖"的最低 col_sum

# 相邻带 cx 跳变最大值：圆环切线斜率上限 + sensor 抖动余量。
# 30 px / (35 px 带间距) = tan ≈ 0.86，对应 ~40°，足够容忍最严的圆切线。
DELTA_CX_MAX_PX = 30

# ---------------------------------------------------------------------------
# 阶段 B：L1 形态学后端
# ---------------------------------------------------------------------------
# "image_open"：二值图 wrap 成 image.Image(ALLOC_REF) 后调 img.open(1)
#               （OpenMV 原生 erode+dilate，3×3/1 iter，~1-2 ms）
# "none"     ：跳过形态学，仅靠 L2 硬约束去噪
# 默认 image_open；如阶段 B 实测 σ(cx) 已达标可切 none 省 ~2 ms / 帧。
LINE_L1_BACKEND = "image_open"

# ---------------------------------------------------------------------------
# 阶段 B：Q_L2 评分（plan §6.6 子集；去掉 IPM/RANSAC 的 geom 与 r_prior）
# ---------------------------------------------------------------------------
# Q_L2 = w_mass * sat(mass_total / MASS_NOMINAL, 0, 1) * 100
#      + w_cont * (1 − jitter_cx / JITTER_REF_PX)      * 100
#      + w_valid * (n_valid / BAND_COUNT)              * 100
# 权重和 = 1.0；jitter_cx 用相邻有效带的 max|Δcx|。
Q_L2_W_MASS = 0.5
Q_L2_W_CONT = 0.3
Q_L2_W_VALID = 0.2

# mass_total 名义值：5 条带的 MIN_MASS_PER_BAND 之和的 ~3 倍（即"每带都达到名义
# 黑线密度"≈ 8 × 20 × 255 = 40800 / 带，5 条 = 204000）。取 100000 作为饱和点
# 让常态 Q_mass ≈ 80~100。
Q_L2_MASS_NOMINAL_TOTAL = 100000

# jitter_cx 参考值：超过该值的相邻带 cx 跳变直接把 Q_cont 拉到 0。
# 与 DELTA_CX_MAX_PX 一致（>30 已经触发硬约束剔除，参考值放在它下方更敏感）。
Q_L2_JITTER_REF_PX = 20.0

# Q 分级（plan §6.6 + §7.2）。阶段 B 仅展示用，主控不消费；
# 控制律集成在阶段 E 才落地。
Q_GOOD = 80
Q_DEGRADE = 60
Q_HOLD = 40

# ---------------------------------------------------------------------------
# 阶段 B：调试 OSD 颜色（detection 可视化）
# ---------------------------------------------------------------------------
OSD_BAND_COLOR = (0, 255, 255)        # 扫描带边框（青）
OSD_CX_VALID_COLOR = (0, 255, 0)      # 有效带 cx 圆点（绿）
OSD_CX_INVALID_COLOR = (255, 0, 0)    # 无效带 cx 圆点（红）
OSD_WIDTH_COLOR = (255, 255, 0)       # 等效宽度水平短线（黄）
OSD_CX_RADIUS_PX = 4                  # cx 圆点半径

# 二值图叠加：把 bin_inv_roi（前景=黑线像素=255）以红色绘制到 OSD 上，
# 肉眼看到的"红色斑块"就是算法当前判定为黑线的像素，便于排查阈值偏差、
# 阴影误检、形态学开运算前后的差异。
# 实现：不依赖 osd.draw_image(mask=...)（K230 上该组合静默不显示）；
# camera.py 用 ``bytes(binary_np)`` 一次物化为 Python bytes 后扫描 ROI，
# 按前景像素绘制 OSD 矩形。bytes 索引 ~50ns，远快于 ulab ``row[x]``
# ~100us。
DEBUG_SHOW_BINARY = True               # 主画面 ROI 红色 overlay 总开关
DEBUG_SHOW_BINARY_PREVIEW = False       # 右上角原尺寸黑白预览独立开关
OSD_BINARY_COLOR = (255, 0, 0)         # 黑线像素叠加色（红）
# K230 image.draw_rectangle 签名 (x,y,w,h,color,thickness,fill)，**不接受
# alpha 参数，也不支持 4 元组 RGBA**（见 docs/k230_canmv_docs/api/openmv/
# image.md `draw_rectangle`）。OSD ARGB8888 真 alpha 通道唯有手写 buffer
# 才能改，但前面已记录 K230 ulab↔image cache 一致性问题。
# 因此"半透明"只能靠抖动（dithering）模拟——OSD_BINARY_ALPHA 仅为兼容字段。
OSD_BINARY_ALPHA = 220
# overlay 渲染模式（主画面 ROI 红色高亮）：
#   "bands_only"     : 仅 5 条 L2 扫描带画不透明红色（覆盖 ROI ~27%）；
#                      CPU 最低，但只能看到 5 条独立带。
#   "full_solid"     : 全 ROI 画不透明红色矩形（覆盖完整，连续无缝隙）；
#                      视觉冲击大，但能直接看出"哪些像素 < threshold"。
#   "full_dither_50" : 全 ROI 棋盘格 (x+y)%2==0 → 50% 红色密度，伪半透明；
#                      ~24000 个潜在 1x1 OSD 矩形（前景率 ~1-3% → ~360 个）。
#   "full_dither_25" : 全 ROI 2x2 块取 1 → 25% 红色密度；红斑更轻，
#                      OSD 矩形数量减半。
#   "full_dither_12" : 全 ROI 4x2 块取 1 → 12.5% 红色密度；最稀疏，
#                      OSD 矩形数量再减半，适合 BINARY_REFRESH 提到每帧时。
#   "full_dither"    : 兼容旧名，等价 "full_dither_50"。
OSD_BINARY_OVERLAY_MODE = "full_dither_12"

# 二值 overlay 刷新间隔（ms）：
#   0   = 每帧刷新（与 algo FPS 同步；红斑实时跟随线缆）；
#   33  = ~30 Hz；
#   100 = 10 Hz；
#   1000 = 1 Hz（与 OSD 文字行同频，最省 CPU）；
# 文字 / FPS / 内存等行仍按 OSD_REFRESH_INTERVAL_MS（1Hz）刷新；
# binary overlay 独立提频不会导致文字 / ROI 框抖动——render_overlay
# 始终带着上一次的 lines 缓存做整体重画。
OSD_BINARY_REFRESH_MS = 0
# bands_only 模式的抽样步长（仅在 OSD_BINARY_OVERLAY_MODE="bands_only" 时生效）。
OSD_BINARY_STRIDE_X = 1
OSD_BINARY_STRIDE_Y = 2
# preview（右上角窗）的抽样步长。预览本来就 320×150 小图，stride=(2,2)
# 足够看出形状又能省 OSD 图元数。
OSD_BINARY_PREVIEW_STRIDE_X = 2
OSD_BINARY_PREVIEW_STRIDE_Y = 2
OSD_BINARY_MIN_RUN_PX = 2              # 过滤单像素噪点（仅 bands_only 用）

# ---------------------------------------------------------------------------
# 标定文件
# ---------------------------------------------------------------------------
# 完整标定（IPM/内参/光度合一）—— 阶段 C 才落到这里。
CALIB_PATH = "/sdcard/calib.json"
# 阶段 B 的独立光度标定脚本写盘路径（plan §11.1 tools/）。不污染 calib.json。
PHOTO_CALIB_PATH = "/sdcard/calib_photometric.json"


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
