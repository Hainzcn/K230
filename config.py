"""K230 视觉循迹子系统配置。

阶段 A 阶段：仅包含摄像头采集、显示与调试叠加所需的最小配置项；
后续阶段（光度、IPM、控制律、UART）会按 vision_line_tracking_plan_v2.md
§11 的目录结构持续扩展，本文件保持唯一配置源（single source of truth）。

约定：
- 所有物理量变量后缀带单位（如 ``*_mm``、``*_ms``、``*_px``）。
- 任何阈值/增益必须先在此处声明，禁止脚本内硬编码（plan §11.2）。
- 标定结果统一存放在 ``calib.json``，由 :func:`load_calibration` 在启动期载入。
"""

CONFIG_VERSION = "phaseD-0.1"

# ---------------------------------------------------------------------------
# 显示设备
# ---------------------------------------------------------------------------
# CanMV K230D Zero 默认 ST7701 800×480 LCD（与 camera_single_bind_lcd.py 一致）。
DISPLAY_TYPE = "ST7701"
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TO_IDE = True

# ---------------------------------------------------------------------------
# Sensor 硬件接口
# ---------------------------------------------------------------------------
# CSI0 接口上外接 OV5647 摄像头（已弃用板载摄像头）。
# Sensor(id=SENSOR_ID, ...) 中 id 对应开发板 CSI 端口编号（0-2）。
# 驱动日志会打印 ``find sensor ov5647_csiN`` 中的 N 即为此处的值。
SENSOR_ID = 0   # CSI0，对应新接入的 OV5647

# 焦点配置（软件路径）。
# 注意：K230 CanMV 的 OV5647 驱动未注册 VCM 驱动，focus_caps() 返回
# (0,0,0)，focus_pos() 为空操作。当前焦距须物理旋转镜头筒后锁定。
# 若未来固件加入 VCM 支持，下列参数会自动生效：
#   SENSOR_AUTOFOCUS = True   启用自动对焦
#   SENSOR_AUTOFOCUS = False  手动定焦，焦点位置由 SENSOR_FOCUS_POS 指定
#   SENSOR_FOCUS_POS          0（无穷远）~ maxPos（最近），赛车近距取大值
SENSOR_AUTOFOCUS = False
SENSOR_FOCUS_POS = 1000    # 极近端备用；当前由物理调焦实现

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
# 驱动日志形如 ``find sensor ov5647_csi0, output WxH@FPS``；如果日志显示的
# W×H 不等于这里的 REQ 值，说明驱动没匹配到，自动回落到默认 1920×1080，而
# 该默认模式在 OV5647 上最高只能 30 FPS。
#
# OV5647 支持的 (w, h, fps)：
#   2592×1944@10 / 1920×1080@30 / 1280×960@45 / 1280×720@60 / 640×480@90
# GC2093 支持的 (w, h, fps)：
#   1920×1080@30/60 / 1280×960@60 / 1280×720@90
#
# 采用低分辨率高帧率模式（OV5647 CSI0）：
#   640×480@90 FPS — 传感器原生最高帧率模式。
#   此模式下 CHN0（显示通道）输出不能超过 640×480，因此 SENSOR_DISPLAY_W/H
#   设置为 640×480（在 800×480 LCD 上居左显示，两侧有黑边，仅作调试用）。
#   CHN1（算法通道）320×240 仍可从 640×480 源正常下采样。
SENSOR_REQ_WIDTH = 640
SENSOR_REQ_HEIGHT = 480
SENSOR_NOMINAL_FPS = 90

# 显示通道（CHN0）输出尺寸：不得超过传感器原生分辨率（SENSOR_REQ_W/H）。
# 当传感器原生 < LCD（800×480）时设为传感器原生，LCD 两侧留黑边。
SENSOR_DISPLAY_X = 0
SENSOR_DISPLAY_Y = 0
SENSOR_DISPLAY_W = 640
SENSOR_DISPLAY_H = 480

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

# OSD 内存告警阈值：这里的 free/alloc 只来自 MicroPython 堆，不代表开发板
# 1GB DDR / MMZ 总资源；drift 看 mem_free 震荡，low-free 只在堆余量紧张时提示。
# K230 运行期稳定 free 常在数 MB 级，不能把 4 MB 当作低内存阈值，否则会常驻告警。
MEM_DRIFT_ALERT_PCT = 5.0            # 漂移超过 5% 提示（阶段 A 硬指标是 ≤10%）
MEM_LOW_ALERT_BYTES = 512 * 1024     # 剩余堆低于 512 KB 才提示

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
LINE_THRESHOLD_BIAS = 10
# - LINE_THRESHOLD_MIN / MAX：硬下限 / 上限。Otsu 在病态场景偶尔会给出极低
#   或极高值（例如全场都是黑色阴影时给 < 20），下限保证黑色目标不会因此完全
#   不被检出；上限避免过度饱和把背景一起判作前景。
LINE_THRESHOLD_MIN = 20
LINE_THRESHOLD_MAX = 80

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
# 阶段 B：L2 硬约束（plan §6.2）—— profile 切换
# ---------------------------------------------------------------------------
# Plan §6.2 默认阈值是按"装车后近处看到 18 mm 电工胶带的 IPM 投影"标定的：
#   每带 mass ≈ 8 行 × 20 列 × 255 = 40800（取 25% 当 MIN ≈ 10000）；
#   width  ≈ IPM 后近处 20 px、远处 10 px。
# 桌面调试场景（黑色线缆 / 显示器边框 / 不装车）下黑色目标的几何完全不同：
#   线缆直径 ~3-5 mm，圆柱中间反光只剩两条 1-2 px 的边线；ROI 内前景占比
#   < 1%（实测 0.7%），mass / width 全部撞硬约束，V=0/5 是必然结果。
#
# 解决：加 ``LINE_DETECTION_PROFILE`` 在 "bench"（桌面调试，宽容）和
# "track"（装车赛道，plan 默认）之间切换。下方 _XXX_BENCH / _XXX_TRACK
# 是真实数值，启动时按 profile 选一组绑定到无前缀的 config 名上。
LINE_DETECTION_PROFILE = "bench"

# ----- track：装车赛道默认（plan §6.2） ----- #
_MIN_MASS_PER_BAND_TRACK = (4000, 5000, 6500, 8000, 10000)
_W_MIN_PX_PER_BAND_TRACK = (5, 6, 8, 10, 12)
_W_MAX_PX_PER_BAND_TRACK = (16, 18, 22, 26, 30)
_COL_SUM_THR_FOR_WIDTH_TRACK = 255   # 严格 > ：每列至少 ≥ 2 个前景像素才计数

# ----- bench：桌面调试（线缆 / 显示器边框 / 走线槽） ----- #
# 比 track 松一个数量级；目标只是验证 detector 主干通路 + L2 硬约束没漏。
# 数值由 0.7% 前景占比反推：89 px / 5 带 ≈ 18 px/带 → mass ≈ 4500，
# 但实际多集中在 1-2 条带，单带 mass 可能 ~10000，其他带 < 500 → MIN 设到
# 几百量级让"有信号的带"过、"没信号的带"挂。COL_SUM_THR=0 让单像素列也
# 算 width；W_MIN=1 容忍线缆边线只剩 1 px。
#
# **几何约束（width 上限 / dcx）在 bench 直接禁用**：桌面摆放的黑线缆 / 走线槽
# 朝向任意——水平横放时 col_sum 在 320 列都 > 0，width_px≈320；斜放时 width
# 也轻松到 50+。这些都和 plan §6.2 假设的"装车后 IPM 投影 18 mm 胶带"完全
# 不同，width 上限失去物理意义。把 W_MAX 全设为 ROI_W、DELTA_CX_MAX 设到大
# 数即可让 bench 模式仅用 ``MIN_MASS_PER_BAND`` 一项做过滤，保持 detector
# 主干（L0/L1/L2 列向求和 + cx 计算）的通路验证。
_MIN_MASS_PER_BAND_BENCH = (300, 400, 500, 700, 900)
_W_MIN_PX_PER_BAND_BENCH = (1, 1, 1, 1, 1)
_W_MAX_PX_PER_BAND_BENCH = (320, 320, 320, 320, 320)   # 等同禁用宽度上限
_COL_SUM_THR_FOR_WIDTH_BENCH = 0     # ≥ 1 个前景像素的列就计数（含 col_sum=255 单像素）

# Profile 解析：默认 track 防止"装车后忘记切回去"。
if LINE_DETECTION_PROFILE == "bench":
    MIN_MASS_PER_BAND = _MIN_MASS_PER_BAND_BENCH
    W_MIN_PX_PER_BAND = _W_MIN_PX_PER_BAND_BENCH
    W_MAX_PX_PER_BAND = _W_MAX_PX_PER_BAND_BENCH
    COL_SUM_THR_FOR_WIDTH = _COL_SUM_THR_FOR_WIDTH_BENCH
else:
    MIN_MASS_PER_BAND = _MIN_MASS_PER_BAND_TRACK
    W_MIN_PX_PER_BAND = _W_MIN_PX_PER_BAND_TRACK
    W_MAX_PX_PER_BAND = _W_MAX_PX_PER_BAND_TRACK
    COL_SUM_THR_FOR_WIDTH = _COL_SUM_THR_FOR_WIDTH_TRACK

# 相邻带 cx 跳变最大值：圆环切线斜率上限 + sensor 抖动余量。
# 30 px / (35 px 带间距) = tan ≈ 0.86，对应 ~40°，足够容忍最严的圆切线。
# bench 模式下桌面线缆 / 走线槽朝向任意，把 dcx 上限放到 ROI 全宽，
# 等同禁用 dcx 检查；bench 仅靠 ``MIN_MASS_PER_BAND`` 过滤每条带。
#
# **phaseB-0.2 起语义改变**：之前 ``DELTA_CX_MAX_PX`` 是事后剔除（band j
# 的 cx 与 band j-1 已选 cx 差超过该值则丢弃 band j）；现在用作"段选择
# 阶段的空间 prior 半径"——band j 的候选段必须落在"上一带已选 cx ±
# DELTA_CX_MAX_PX"内才参与选段排序，干扰物在 prior 之外直接出局，
# 不再"拉偏全列质心后再剔除"。同一个数值，更早起作用。
DELTA_CX_MAX_PX = 320 if LINE_DETECTION_PROFILE == "bench" else 30

# ---------------------------------------------------------------------------
# 阶段 B (phaseB-0.2)：段查找 + 时空 prior（plan §6.2 抗干扰升级）
# ---------------------------------------------------------------------------
# 背景：旧 cx 计算是 ``cx = Σx·col_sum / Σcol_sum``——ROI 内有任何前景像素
# 都按质量加权进入平均值。一旦 ROI 出现"另一块黑"（路面碎屑 / 阴影 / 桌面
# 异色），cx 会被拉到"主黑线 + 干扰"二者的质心位置，控制律按这个跟踪相当于
# 直接撞过去。早期 task_log §4 的 ``[ ] 阴影 / 反光抗干扰升级`` TODO 就是
# 为此预留。
#
# 反光问题（黑线中间镜面反光打洞、把单段拆成两段）由镜头加偏振片在硬件侧
# 处理；软件侧只解决"干扰物体"。
#
# 新数据流：col_sum → find_runs(>COL_SUM_THR) → 候选筛选(W/MASS) →
# 选段排序(时域 prior → 空间 prior → mass 兜底) → 选中段的 cx_seg/width。
# 详见 vision/line_detector._find_runs_from_bytes / _select_best_run。

# 桥接窄洞：连续段查找时，允许 ≤ N 列的"低于 COL_SUM_THR_FOR_WIDTH"被
# 视作段内（防 sensor 噪声 / 抗锯齿把单段断成两短段，让 W_MIN 过滤失误）。
# 0  = 严格（旧行为）；
# 1  = 允许 1 列窄洞（推荐默认；电气胶带边缘 1 px 抗锯齿常见）；
# 2+ = 更宽松，但会把"两条相距 ≤ N 的窄黑线"误连成一条。
LINE_RUN_GAP_TOLERANCE_PX = 1

# 时域 prior：上一帧本带选中的 cx 作为本帧选段时的最优先排序依据。
# 半径外的候选段被时域 prior 排除；半径内的按 |cx - prev_cx| 升序排序。
# 默认复用 DELTA_CX_MAX_PX，避免再开旋钮。
LINE_CX_PRIOR_RADIUS_PX = DELTA_CX_MAX_PX

# 时域 prior 失效阈值：cx_prev 在该帧数后视为过期（不参与选段）。
# 30 FPS 下 5 帧 ≈ 167 ms。短于人眼反应时间，能容忍偶发遮挡 / 单帧丢检；
# 长时间真丢线时不让陈旧 prior 把检测拉回错误区域。
LINE_CX_PRIOR_AGE_MAX_FRAMES = 5

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

# mass_total 名义值：track 模式按 5 条带 × MIN_MASS_PER_BAND × 3 倍估的
# （每带 IPM 投影下 8 × 20 × 255 = 40800，5 条饱和值 ≈ 200000）；bench
# 模式黑线缆 ROI 占比 < 1%，整帧 mass_total 通常 5000~20000，按此放缩
# nominal 到 10000，让 Q_mass 在桌面"看到线"时也能给出 ~70 分。
if LINE_DETECTION_PROFILE == "bench":
    Q_L2_MASS_NOMINAL_TOTAL = 10000
else:
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
# debug_overlay.py 用 ``bytes(binary_np)`` 一次物化为 Python bytes 后扫描 ROI，
# 按前景像素绘制 OSD 矩形。bytes 索引 ~50ns，远快于 ulab ``row[x]``
# ~100us。
DEBUG_SHOW_BINARY = True               # 主画面 ROI 红色 overlay 总开关
DEBUG_SHOW_BINARY_PREVIEW = False       # 右上角原尺寸黑白预览独立开关

# 外接按键：控制主画面 ROI 红色 overlay 运行期开关。
# 接线：IO_42（开放引脚 Header NO.9）-- 按键 -- GND；软件启用上拉。
DEBUG_BINARY_BUTTON_ENABLE = True
DEBUG_BINARY_BUTTON_IO = 42
DEBUG_BINARY_BUTTON_ACTIVE_LOW = True
DEBUG_BINARY_BUTTON_DEBOUNCE_MS = 80

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
#   100 = 10 Hz（推荐默认；肉眼仍能看出二值区域变化，明显减轻 OSD 压力）；
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
# 阶段 C：IPM 安装几何（plan §4.1，calib.json 缺失时用作 fallback H 推导）
# ---------------------------------------------------------------------------
# 实测装车后必须用 tools/calibrate_ipm.py 解出 H 写到 calib.json，覆盖此处。
# 这里只是为了让代码骨架在装车前桌面 bench 上也能跑通全链路（OSD 会显式
# 标 "CALIB:DEFAULT" 提醒用户：此时 e_y / ψ_e 数值有几十 mm 系统偏差）。
MOUNT_H_CAM_MM = 120                   # 镜头离地高度（plan §4.1 80~150 mm）
MOUNT_PITCH_DEG = 20.0                 # 光轴俯仰角（向下为正；plan §4.1 15~25°）
MOUNT_YAW_DEG = 0.0                    # 光轴偏航（实测 < 1°）
# OV5647 + 标准镜头（CanMV 套件默认）水平 FOV 约 54°；横纵 FOV 与
# 1280×720 上的焦距 fx/fy 反推：fx = (W/2) / tan(HFOV/2)。下值仅作 fallback
# 用，标定脚本里会覆盖。
SENSOR_HFOV_DEG = 54.0
SENSOR_VFOV_DEG = 41.0

# IPM 后近处一行 / 远处一行的 mm/px 估值（plan §4.1 输出）。仅在 fallback
# 模式下供 ground_mapper 用作车体坐标系输出的 sanity check 与 OSD 调试；
# 标定模式下从 H 矩阵直接换算。
MM_PER_PIX_NEAR = 1.0                  # ≈ NEAR 行 1 mm/px（视野较密）
MM_PER_PIX_FAR = 4.0                   # ≈ FAR 行 4 mm/px（视野较疏）

# ---------------------------------------------------------------------------
# 阶段 C：RANSAC 圆弧拟合（plan §6.3）
# ---------------------------------------------------------------------------
# 黑线中心线半径先验：(800 mm 内径 + 18 mm 黑线宽度) / 2 = 409 mm。
R_PRIOR_MM = 409.0
# 半径先验容差：超出 ±50 mm 直接丢弃假设（plan §6.3）。
R_PRIOR_TOL_MM = 50.0
# 内点判据：圆上点到拟合圆的距离 ≤ ε 视为 inlier（plan §6.3）。
RANSAC_INLIER_EPS_MM = 10.0
# 至少 N 个 inlier 才视为成功（plan §6.3 "最小样本 3"）。
RANSAC_MIN_INLIERS = 3
# 5 个候选点 → C(5,3)=10 个 3-元组，枚举即可，无需随机迭代。
# 若样本数 < MIN_SAMPLES，整帧 RANSAC 直接放弃，调用方走 L3b 直线 fallback。
RANSAC_MIN_SAMPLES = 3

# ---------------------------------------------------------------------------
# 阶段 C：估计器（plan §7.4 EMA；一维 Kalman 留待阶段 E 之后）
# ---------------------------------------------------------------------------
EMA_ALPHA_E_Y = 0.5                    # plan §7.4 推荐 0.4~0.6
EMA_ALPHA_PSI = 0.5
# valid=False 时 EMA 保持上一帧；连续 N 帧失效后 reset()，避免陈旧值卡死控制律。
EMA_AGE_MAX_FRAMES = 5

# 符号防抖：sign(e_y) 翻转需连续 N 帧同号才接受（plan §8.1）。
# 30 FPS 下 3 帧 ≈ 100 ms，足以抑制单帧抖动诱发的方向反转。
SIGN_FLIP_DEBOUNCE_FRAMES = 3

# ---------------------------------------------------------------------------
# 阶段 C：Q_full 评分（plan §6.6 完整版；与 Q_L2 共存，bench 模式仍可只看 Q_L2）
# ---------------------------------------------------------------------------
# Q_full = w_mass · sat(mass_total / NOMINAL, 0, 1)        · 100
#        + w_geom · sat(inlier_ratio, 0, 1)                · 100
#        + w_cont · sat(1 − jitter / JITTER_REF, 0, 1)     · 100
#        + w_r_prior · sat(1 − |R̂ − R_PRIOR| / 80, 0, 1)   · 100
# 权重按 plan §6.6 推荐值；总和 1.0。
Q_W_MASS = 0.3
Q_W_GEOM = 0.3
Q_W_CONT = 0.2
Q_W_R_PRIOR = 0.2
# r_prior 子项的归一化半径偏差（plan §6.6 注释里给的 80 mm）。
Q_R_PRIOR_NORM_MM = 80.0

# ---------------------------------------------------------------------------
# 阶段 C：调试 OSD 颜色与几何
# ---------------------------------------------------------------------------
OSD_PATH_COLOR = (0, 255, 0)           # 5 点 cx 折线（绿）
OSD_PATH_THICKNESS = 2
OSD_TANGENT_COLOR = (255, 128, 0)      # 近带切线箭头（橙）
OSD_TANGENT_THICKNESS = 2
OSD_TANGENT_LEN_PX = 30                # 算法分辨率下的箭头长度
OSD_CIRCLE_CENTER_COLOR = (255, 0, 255)  # 圆心反投点（品红）
OSD_CIRCLE_CENTER_RADIUS_PX = 6
OSD_CALIB_DEFAULT_COLOR = (255, 200, 0)  # CALIB:DEFAULT 文字（琥珀）
OSD_NO_CALIB_COLOR = (255, 0, 0)         # NO CALIB 文字（纯红）

# ---------------------------------------------------------------------------
# 标定文件
# ---------------------------------------------------------------------------
# 完整标定（IPM/内参/光度合一）。阶段 C 起 ground_mapper 启动期解析此文件。
# Schema（与 tools/calibrate_ipm.py 写出一致）::
#
#   {
#     "config_version": "phaseC-0.1",
#     "ts_ms": int,
#     "ipm": {
#       "H_3x3": [9 floats, row-major]   ← 像素 (u,v,1) → 地面 (x_g,y_g,1) 单应
#       "image_wh": [320, 240],          ← 标定时使用的算法分辨率，必须匹配
#       "corners_image":  [[u,v]*4],     ← 4 个标靶在图像上的像素坐标（仅记录）
#       "corners_ground_mm": [[x,y]*4]   ← 4 个标靶在地面坐标系上的 mm 坐标
#     },
#     "photometric": {                   ← 可选；photometric.bootstrap 也能填
#       "mu_bg": ..., "sigma_bg": ...,
#       "thr_otsu": ..., "line_threshold": ...
#     }
#   }
#
# 字段缺失时由 ground_mapper / photometric 各自降级（plan §11.2）。
CALIB_PATH = "/sdcard/calib.json"
# 阶段 B 的独立光度标定脚本写盘路径（plan §11.1 tools/）。不污染 calib.json。
PHOTO_CALIB_PATH = "/sdcard/calib_photometric.json"


# ---------------------------------------------------------------------------
# 阶段 D：UART 链路（plan §10）
# ---------------------------------------------------------------------------
# COMMS_ENABLE=False 时主循环跳过所有 UART 初始化（bench 桌面调试用）。
# 装车后设 True，同时确认 IMU_UART_ID / MCU_UART_ID 与硬件接线一致。
#
# 接线（Stage4-K230-Side.md §1）：
#   UART(1, 115200) RX ← MS901M TX 线 Y 分支路
#   UART(2, 921600) TX → MCU PB7 RX；RX ← MCU PB6 TX
#   共 GND（MCU GND → K230 GND），严禁互接 5V 电源
# ---------------------------------------------------------------------------
COMMS_ENABLE = True          # bench 关闭，装车设 True

# UART 通道号（可用：UART1 / UART2 / UART4；UART0=小核SH，UART3=大核SH，不可用）
IMU_UART_ID  = 1              # UART1  115200，接 MS901M TX Y 分线（仅 RX）
MCU_UART_ID  = 2              # UART2  921600，接 MCU UART1（TX+RX 双向）

# ---------------------------------------------------------------------------
# FPIOA 引脚映射（LP_PIN.md 及 IOMUX 手册）
#
# UART2（MCU 命令链路，Header NO.17 / NO.20）：
#   IO_5  = UART2_TXD  ← K230 发 → MCU RX (MCU PB7)
#   IO_6  = UART2_RXD  ← MCU TX (MCU PB6) → K230 收
#
# UART1（IMU 直通，仅 RX，Header NO.13/NO.18 JTAG 引脚复用）：
#   IO_3 = UART1_TXD  → 必须配置，即使不发送（驱动要求 TX+RX 同时 FPIOA）
#   IO_4 = UART1_RXD  ← MS901M TX Y 分线
#
# fpioa.help() 实测：
#   IO_3 可用功能：GPIO3/JTAG_TDI/PULSE_CNTR1/UART1_TXD/RESV  (Header NO.18)
#   IO_4 可用功能：GPIO4/JTAG_TDO/PULSE_CNTR2/UART1_RXD/RESV  (Header NO.13)
# JTAG 在正常运行期间不占用这两个引脚，可安全复用为 UART1。
# ---------------------------------------------------------------------------
MCU_UART2_TX_IO  = 5          # IO_5  → UART2_TXD (Header NO.17)
MCU_UART2_RX_IO  = 6          # IO_6  → UART2_RXD (Header NO.20)
IMU_UART1_TX_IO  = 3          # IO_3  → UART1_TXD (Header NO.18，驱动要求配置，TX 实际不用)
IMU_UART1_RX_IO  = 4          # IO_4  → UART1_RXD (Header NO.13)

# MCU 心跳超时：超过此时长未收到 HEARTBEAT_MCU → K230 进入降级（发 mode=0）
# plan §2.2 规定 MCU 侧 200 ms 未收 MOTION_CMD 就平衡保护；K230 侧放宽到
# 500 ms 触发降级，给通讯抖动预留余量。
MCU_TIMEOUT_MS = 500

# K230 → MCU 心跳发送间隔（ms）：必须 < MCU_TIMEOUT_MS。
# MCU 侧如果 500 ms 内收不到任何帧就归零；心跳按 400 ms 发，留 100 ms 裕量。
HB_SEND_INTERVAL_MS = 400

# MOTION_CMD 发送频率：25 ms = 40 Hz（plan §4.2 K230→MCU 20~50 Hz）。
CMD_SEND_INTERVAL_MS = 25

# 低电量减速阈值：bat_mv 低于该值时 K230 把 target_v 钳位到 v_degrade，
# 对应 MCU 侧 LOW_BAT_WARN 提前量，减少紧急停车风险（Stage4 §6 规则）。
BAT_DEGRADE_MV = 9500

# 低电量时纵向速度上限（counts/s / SCALE；同 MOTION_CMD target_v 单位）。
BAT_DEGRADE_V_MAX = 200

# MOTION_CMD 目标速度 / 转向默认值（Stage E 控制律落地前占位）。
# target_v     : 0 = 原地平衡，Stage E 后由控制律填充
# target_omega : 0 = 不转向
MOTION_DEFAULT_V     = 0
MOTION_DEFAULT_OMEGA = 0


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


def _safe_open_text(path):
    """K230 / CPython 兼容的文本读取；任何异常都吞掉返回 None。"""
    try:
        with open(path, "r") as f:
            return f.read()
    except OSError:
        return None
    except Exception as e:
        print("[config] load_calibration: open %s failed: %s" % (path, e))
        return None


def _validate_ipm_node(ipm):
    """轻校验 IPM 节，返回 (ok, error_msg)。不抛异常。"""
    if not isinstance(ipm, dict):
        return False, "ipm node not dict"
    h = ipm.get("H_3x3")
    if not isinstance(h, (list, tuple)) or len(h) != 9:
        return False, "ipm.H_3x3 must be list of 9 floats"
    for v in h:
        if not isinstance(v, (int, float)):
            return False, "ipm.H_3x3 contains non-numeric"
    wh = ipm.get("image_wh")
    if wh is not None:
        if not isinstance(wh, (list, tuple)) or len(wh) != 2:
            return False, "ipm.image_wh must be [W, H]"
        if int(wh[0]) != ALGO_WIDTH or int(wh[1]) != ALGO_HEIGHT:
            return False, (
                "ipm.image_wh=%sx%s mismatch ALGO=%dx%d"
                % (wh[0], wh[1], ALGO_WIDTH, ALGO_HEIGHT)
            )
    return True, ""


def load_calibration(path=None):
    """启动期加载 ``calib.json``。

    返回 dict；缺失或解析失败时返回 ``{}``，并在控制台打印诊断行。
    调用方（``vision.ground_mapper.GroundMapper.load`` 等）按 dict 是否含
    ``ipm.H_3x3`` 决定走标定 / fallback / NO CALIB 三档。

    禁止抛异常：plan §11.2 规定模块自检不可阻塞主循环。
    """
    try:
        import ujson as json  # K230 优先
    except ImportError:
        import json
    if path is None:
        path = CALIB_PATH

    text = _safe_open_text(path)
    if text is None:
        print("[config] load_calibration: %s not found, using fallback" % path)
        return {}

    try:
        payload = json.loads(text)
    except Exception as e:
        print("[config] load_calibration: JSON parse failed: %s" % e)
        return {"error": "json_parse"}

    if not isinstance(payload, dict):
        print("[config] load_calibration: top-level not dict")
        return {"error": "not_dict"}

    ver = payload.get("config_version")
    if ver and ver != CONFIG_VERSION:
        print(
            "[config] load_calibration: WARNING calib version=%s vs current=%s"
            % (ver, CONFIG_VERSION)
        )

    ipm = payload.get("ipm")
    if ipm is not None:
        ok, msg = _validate_ipm_node(ipm)
        if not ok:
            print("[config] load_calibration: ipm rejected: %s" % msg)
            payload["ipm"] = None
            payload["error"] = "ipm_invalid:" + msg

    return payload
