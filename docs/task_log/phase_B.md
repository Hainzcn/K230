# 阶段 B 任务日志：光度标定与多扫描带检测

> 对照计划：`docs/vision_line_tracking_plan_v2.md` §12 阶段 B
> 起止：2026-05-04 ~
> 负责人：—
> 当前状态：**代码完成；性能 + 验收实测待物理装配后回填**

---

## 1. 任务清单与状态

| # | 任务 | 计划文档锚点 | 状态 | 交付物 |
|---|------|--------------|------|--------|
| B-1 | 实现 `photometric.py`：Otsu + bootstrap + 运行期漂移监测 | §5.3 | ✅ 已完成 | `vision/photometric.py` |
| B-2 | 实现 `line_detector.py`：L0 + L1 + L2（5 条扫描带 + 硬约束） | §6.1 / §6.2 | ✅ 已完成 | `vision/line_detector.py` |
| B-3 | 实现 `quality.py`：Q_L2（仅含 L2 子项） | §6.6 | ✅ 已完成 | `vision/quality.py` |
| B-4 | 调试叠加：扫描带 / cx 圆点 / 宽度 / Q_L2 | §12 阶段 B | ✅ 已完成 | `vision/camera.py` `_draw_detection` |
| B-5 | 独立光度标定脚本 | §5.3 / §11.1 | ✅ 已完成 | `tools/calibrate_photometric.py` |
| B-6 | 主入口装配 photometric + detector | §11.3 | ✅ 已完成 | `vision_line_tracking.py` |
| B-7 | 静止状态 σ(cx) 实测 | §12 阶段 B 验收 | ⏳ 待装车 | — |
| B-8 | 光度 ±30% 变化 < 1 s 收敛实测 | §12 阶段 B 验收 | ⏳ 待装车 | — |
| B-9 | 阴影边缘误检率实测 | §12 阶段 B 验收 | ⏳ 待装车 | — |

---

## 2. 代码侧实现纪要

### 2.1 文件结构（与计划书 §11.1 一致）

```
K230/
├── config.py                          # CONFIG_VERSION 升 phaseB-0.1
├── tools/
│   └── calibrate_photometric.py       # 30 帧 Otsu 标定脚本
├── vision/
│   ├── __init__.py                    # __all__ 加 photometric / line_detector / quality
│   ├── camera.py                      # render_overlay(lines, detection=)，增 _draw_detection / algo_xy_to_display
│   ├── photometric.py                 # Photometric (阈值 + 漂移监测)
│   ├── line_detector.py               # LineDetector (L0+L1+L2)
│   └── quality.py                     # compute_q_l2 / grade
├── vision_line_tracking.py            # 装配 photometric + detector
└── docs/task_log/phase_B.md           # 本文件
```

### 2.2 核心设计点

#### 2.2.1 算法主路径

```text
snapshot CHN1 (320×240 GRAY)
  └─► cv_lite.grayscale_threshold_binary(thresh=photo.threshold, 255)        # L0
        ├─► bin_raw   (黑线=0  背景=255)
        └─► bin_inv = 255 − bin_raw_roi   (仅 ROI 行；前景=255)
              └─► (LINE_L1_BACKEND=image_open) image.Image(ALLOC_REF) → open(1)   # L1
                    └─► 5 条扫描带 ulab 切片 + col_sum(axis=0)                  # L2
                          ├─► mass / cx / width
                          └─► 硬约束（mass / W / Δcx）→ BandResult
                                └─► quality.compute_q_l2 (mass + cont + valid)
```

每帧调用一次 `LineDetector.process(img, threshold)`，返回内部复用的
`DetectionResult`，避免帧循环 alloc（plan §9.2 守则 5）。

#### 2.2.2 极性约定（黑线 = 前景 = 255）

`cv_lite.grayscale_threshold_binary` 的语义是 `pix > thresh → maxval`，
对暗黑线会输出 0。我们要"黑线 = 前景"才能直接对列求和拿密度，所以
做一次 `bin_inv = 255 − bin_raw_roi`（ulab 标量减；只对 ROI 行做，
省一半 alloc：48 KB vs 76 KB）。

#### 2.2.3 L1 形态学双后端

按"两套都实现"的工程决策（plan 评审的 Q1）：

| `config.LINE_L1_BACKEND` | 实现 | 预算 | 适用 |
|--------------------------|------|------|------|
| `"image_open"`（默认） | `image.Image(ALLOC_REF, data=bin_inv_roi).open(1)` 原地 erode+dilate（3×3 / 1 iter） | ~1-2 ms | 默认开；阴影 / 反光场景去毛刺 |
| `"none"` | passthrough，仅靠 L2 硬约束去噪 | 0 ms | σ(cx) 已达标后省预算 |

之所以放 image.Image 原生开运算而不是 cv_lite：cv_lite 没有
`grayscale_open`，只有 `rgb888_open`，而后者会做 RGB→灰度内部转换，
对我们已经是灰度的输入是浪费。

#### 2.2.4 5 条扫描带的几何

均匀铺在 ROI_TOTAL_PX（y=80~230，150 px 高）：

| i | y_top | y_bot | 角色 |
|---|-------|-------|------|
| 0 | 80    | 88    | FAR  (远带；曲率前馈源) |
| 1 | 116   | 124   | —    |
| 2 | 151   | 159   | MID  (切线方向估计) |
| 3 | 187   | 195   | —    |
| 4 | 222   | 230   | NEAR (e_y 主来源；最近 ROI 底) |

带索引约定：i=0 是最远带，i=BAND_COUNT-1 是最近带。`bands[-1]` 永远
是近带，与 plan §4.3 权重 0.5/0.3/0.2 的概念匹配（融合在阶段 C 才落地）。

#### 2.2.5 L2 硬约束（plan §6.2）

每条带独立判定，单带不合格 → 仅剔除该带，不丢整帧：

| 约束 | 配置项 | 默认（远→近） |
|------|--------|----------------|
| `mass_i ≥ MIN` | `MIN_MASS_PER_BAND` | 4000 / 5000 / 6500 / 8000 / 10000 |
| `W_MIN ≤ width ≤ W_MAX` | `W_MIN/MAX_PX_PER_BAND` | (5,16) / (6,18) / (8,22) / (10,26) / (12,30) |
| `\|Δcx\| ≤ DELTA_CX_MAX_PX` | `DELTA_CX_MAX_PX` | 30 |

宽度近大远小是 IPM 透视下的经验值，装车后实测要校对。`Δcx` 阈值 30 对
35 px 带间距 ≈ tan 40°，足够容忍最严的圆切线 + sensor 抖动。

#### 2.2.6 Q_L2 评分（plan §6.6 子集）

```text
Q_L2 = w_mass  · sat(mass_total / Q_L2_MASS_NOMINAL_TOTAL, 0, 1) · 100   (w=0.5)
     + w_cont  · (1 − jitter_cx / Q_L2_JITTER_REF_PX)             · 100   (w=0.3)
     + w_valid · (n_valid / BAND_COUNT)                           · 100   (w=0.2)
```

`jitter_cx` = 相邻有效带 cx 的最大 |Δcx|；不足 2 条有效带时退化为 0
（不扣 cont 分；valid 项的低分母自然反映"有效带不足"）。

阶段 B 不消费 Q 分级（控制律在阶段 E 才落地），但 OSD / 控制台日志按
plan §7.2 的 80/60/40 三档颜色分级显示，便于眼观即时调试。

#### 2.2.7 光度自适应

bootstrap（启动期、阻塞）：

1. 在 `camera.start()` 之后、主 `while` 之前跑 30 帧；
2. 每帧用 `image.get_histogram(roi=ROI_TOTAL_PX, bins=256)` →
   `histogram.get_threshold().value()` 拿单帧 Otsu，再 `histogram.get_statistics()`
   拿 mean / stdev；
3. finalize 时取均值：`thr_otsu = mean(otsu_30)`、`μ_bg = mean(mean_30)`、
   `σ_bg = mean(std_30)`；
4. `thr_fb = μ_bg − 3·σ_bg`，`line_threshold = (thr_otsu + thr_fb) / 2`。

漂移监测（运行期、不阻塞）：

- 平稳态：每 1 s 用 `image.get_statistics(roi=ROI_TOTAL_PX)` 拿 ROI mean；
  若 `|Δμ_bg| > 15` 进入复算态。
- 复算态：之后 30 次 `update()` 把样本累入累加器；累齐 finalize，
  热替换 `line_threshold`。OSD 出现 `PHOTO recal i/30` 红色提示。

bootstrap 与运行期复算共享同一份 `_accumulate_one` / `_finalize_recalib`，
不重复实现。Otsu 直接复用 K230 image 模块原生接口，不在 Python 层
逐桶扫直方图（plan §9.2 守则 3）。

#### 2.2.8 调试 OSD 增强

`Camera.render_overlay(lines, detection=None)`：

- ROI 框（沿用阶段 A）；
- `_draw_detection(detection)`：5 条带边框（青）+ 每带 cx 圆点（valid 绿、
  invalid 红，半径 4 px）+ 等效宽度水平短线段（黄，cx ± width/2）；
- 文本行新增 `Q V cxN thr` 一行，Q < 40 时整行红；
- 复算态额外追加红色 `PHOTO recal i/N` 行；
- 算法坐标 → 显示坐标用 `algo_xy_to_display` 等比例缩放（与 ROI 一致）。

#### 2.2.9 控制台 5 s 节流日志

```
[VLT] algo_fps=33.1 period=30.2ms frames=1623 Q=82.3(good) V=5/5
      cxN=159.4 cxF=158.7 thr=82 mu=124.0 sig=18.5 mem=... drift=...
```

完整指标按 plan §13.1 落入日志，便于离线 CSV 解析。`RECAL` 标签出现
在复算态。

### 2.3 与阶段 A 的差异

| 维度 | 阶段 A | 阶段 B |
|------|--------|--------|
| 算法层数 | 仅 snapshot | L0 + L1 + L2 + Q_L2 |
| 阈值来源 | 无 | 30 帧 Otsu bootstrap + 1 s 漂移监测 |
| OSD | FPS / period / mem | + 5 条扫描带 / cx 圆点 / 宽度 / Q / recal |
| 主循环步数 | 1 (snapshot) | 4 (snapshot → photometric.update → detector.process → render) |
| 帧预算 (plan §9.1 v2.1) | snapshot ~30 ms (硬上限) | snapshot ~30 ms + 算法 ≤ 17 ms |

---

## 3. 验收记录占位（实测后回填）

> 阶段 B 验收（plan §12）：
> - 静止 cx 抖动 σ ≤ 2 px（近带）、≤ 3 px（远带）；
> - 光度 ±30% 变化下仍能识别；阈值自适应 < 1 s 收敛；
> - 阴影边缘不误检为黑线（面积约束过滤 ≥ 95% 误检）。

| 测试场景 | 配置 | 实测 σ(cx) NEAR | 实测 σ(cx) FAR | 平均 Q_L2 | 通过 |
|---------|------|------------------|------------------|-----------|------|
| 静止 500 帧（L1=image_open） | 默认 | _____ px | _____ px | _____ | ☐ |
| 静止 500 帧（L1=none） | 关 L1 | _____ px | _____ px | _____ | ☐ |
| 光度 ±30% 阶跃 | 手遮光 / 撤光 | 收敛时间 _____ ms | μ_bg 复算 _____ | recal 触发 ☐ | ☐ |
| 阴影边缘 100 帧 | ROI 跨阴影分界 | 误检带数 _____ / 500 | 过滤率 _____% | — | ☐ |
| 整段流水线（带 OSD） | DEBUG=ON | algo_fps _____ | algo_period _____ ms | — | ☐ |
| 整段流水线（无 OSD） | DEBUG=OFF | algo_fps _____ | algo_period _____ ms | — | ☐ |

测试方法：

1. 装车后在静态赛道（黑线在镜头视野中央）启动 `vision_line_tracking.py`；
2. 等待 30 s 进入稳态后开启采样模式 `CAPTURE_ENABLE=True` 录 200 帧；
3. 离线分析 cx_near / cx_far 序列，计算 σ；
4. 手遮光 ±30%（用半透明白纸 / 黑布），记录 OSD `PHOTO recal i/30`
   出现到消失的时间；
5. 把镜头对准阴影边缘（黑线一半进阴影、一半进光区），统计 100 帧里
   `valid 带数 < 5` 的比例，作为误检过滤率代理；
6. 切换 `LINE_L1_BACKEND` 在 `image_open` / `none` 之间各跑一次。

---

## 4. 已知问题与遗留 TODO

- [ ] **mass_total 名义值的标定**：`Q_L2_MASS_NOMINAL_TOTAL=100000` 是按
  "5 条带 × 8 行 × 20 px × 255 / 10" 估的，装车后要用静态稳态的实际
  `mass_total` 中位数重新校准，使 `Q_L2 ≈ 80~95` 在正常工况成立。
- [ ] **W_MIN/W_MAX 的远近差异**：(5,16)~(12,30) 是按 IPM 透视 + 黑线
  18 mm 的粗估，装车后要在样张上量 5 条带的实际 width 像素，回填
  `W_MIN_PX_PER_BAND` / `W_MAX_PX_PER_BAND`。
- [ ] **阴影 / 反光抗干扰升级**：阶段 B 仅靠 L2 硬约束 + Q_L2 cont 项。
  若阴影边缘误检率 < 95%，下一步在 line_detector 里加 "上一帧 cx 邻域
  ±N px 内才纳入" 的时域 prior（不是 EMA，是空间 prior）。
- [x] **K230 ulab bool / uint8 mask 求和语义不稳定**：line_detector 的
  `np.sum(col_sum > thr)` 在板端可能把 True 当 `255` 而不是 `1` 求和，
  导致 `width_px` 被放大 255 倍，桌面 bench 下即使 `W_MAX=320` 也会误触发
  `w>320`，表现为红色 binary overlay 完整标中线缆但 `V=0/5`。修复：
  若 `width_px > roi_w`，改走 `bytes(col_sum)` 小端解码，直接按列统计
  `width`；仅当 `bytes()` 失败时才保留逐元素 `_scalar_to_int()` 兜底。
- [x] **bench profile 几何上限对桌面场景失效**：cx bug 修好之后真实
  `width_px` / `dcx` 终于算对了，桌面线缆 / 走线槽场景反而 V=0/5 全挂
  （前几版 V=5/5 是 width 标量恒 1 的假阳性）。原因：水平 / 斜放黑线缆在
  `band_slice` 8 行内可能贯穿整宽，col_sum 几乎全列 > 0，width_px≈320，
  撞 bench `W_MAX=(40~80)`；线缆斜放时相邻带 cx 差 100+，撞 bench
  `DELTA_CX_MAX=60`。
  修复：bench profile 把 `_W_MAX_PX_PER_BAND_BENCH` 全设为 320、
  `DELTA_CX_MAX_PX (bench)` 设到 320、`_W_MIN_PX_PER_BAND_BENCH` 全
  设为 1。等同**bench 模式只用 `MIN_MASS_PER_BAND` 一项过滤**——此为
  设计预期：bench 仅验证 detector 主干（L0/L1/L2 + cx 计算）通路，几何
  约束在 track 模式才生效（plan §6.2 假设的 IPM 投影 18 mm 胶带 width）。
- [x] **K230 ulab `np.sum(arr, axis=0)` 忽略 axis 参数**：实测对 (8, 320)
  shape 的 ROI 切片调 `np.sum(band_slice, axis=0)`，**返回标量总和而不是
  按列求和的 (320,) 向量**（等价 `np.sum(band_slice)`，axis 被无视）。
  下游连锁反应：
  1. `col_sum` 是 mass 标量；
  2. `mass = float(np.sum(col_sum))` 还是同一个标量值，看着正常；
  3. `cx = sum(arange_x * col_sum) / mass = scalar * sum(arange_x) / mass
     = sum(arange_x) = 0+1+...+(W-1) = 51040.0`（W=320）；
  4. cx_px=51040 喂给 OSD `algo_xy_to_display`，display_x=127600 严重
     超屏，K230 image 模块 `draw_circle / draw_line` 进入慢路径，每帧
     50ms+ → algo FPS 从 33 跌到 13；
  5. bench profile 把 W_MIN 降到 1 才让 V=4-5 通过，bug 才暴露——track
     profile 下 width=1 不通过 W_MIN=5 全部失败 V=0/5，cx_near_px=-1
     不写出，bug 被掩盖。
  修复：`vision/line_detector.process` 用手工逐行累加 `col_sum =
  band_slice[0] + band_slice[1] + ... + band_slice[H-1]` 替代 axis sum，
  再加两道防御 ——
  ① `len(col_sum) != self._roi_w` 时整带置 `col_sum_shape` invalid；
  ② cx 落到 `[0, W)` 之外时整带置 `cx_oob_*` invalid，避免污染 OSD。
  保留 `arr.sum(axis=...)` quirk 作为 plan §15 K230 已知问题清单的一项。
- [x] **K230 ulab 1-D 加权质心仍会广播成 `cx≈51040`**：在修复
  `axis=0` 之后，`col_sum` 已经是长度 320 的向量，但实板仍出现
  `cx = np.sum(arange_x * col_sum) / mass ≈ 51040`（即 `0+1+...+319`），
  日志表现为：
  `Q=50.0(hold) V=0/5`，`[VLT.band] ... cx_oob_51040`，同时
  `mass_total` / `width` 都正常。修复过程：
  1. `vision_line_tracking.py` 增加 `[VLT.band]` 低频诊断行，打印每条带
     `mass / width / cx / reject`，避免只靠 `V=0/5` 猜硬约束；
  2. `line_detector.process` 在 `cx` 越界时不再直接丢带，而是用
     `_col_sum_stats_from_bytes(col_sum, roi_w, threshold)` 从 `bytes(col_sum)`
     小端解码，计算 `weighted=Σx·col_sum[x]` 后重算 `cx`；
  3. 早期逐元素 `_scalar_to_int(v)` 虽能修复 `cx`，但会让 FPS 从 28~30
     暴跌到 6~7；最终改为一次 `bytes(col_sum)` 解析，保留逐元素路径只做
     极端兜底。实测白纸 + 黑色线缆下恢复到 `V=4/5~5/5`，`cxN/cxF`
     落回 `[0,319]` 合法范围。
- [ ] **`tools/calibrate_photometric.py` 仅写独立 JSON**：阶段 C 接 IPM
  时把它合并进 `calib.json` 的 photometric 节，并加 schema 版本号。
- [x] **plan §5.3 fallback 公式在 σ ≳ μ/k 时失效**：早期实现把 `μ−kσ` 钳到
  0 后照样进平均，导致桌面杂物 / 远景非高斯背景下最终阈值被腰斩（实测：
  `μ=57 σ=45 Otsu=68 → threshold=34`，黑线容差仅 13.4%）。修复：当
  `μ−kσ ≤ 0` 或 `≥ Otsu` 时直接退化为 Otsu 单源，并 print 一行
  `[photometric] fallback rejected ...` 诊断；新增 `LINE_THRESHOLD_BIAS` /
  `LINE_THRESHOLD_MIN` / `LINE_THRESHOLD_MAX` 三个收尾旋钮供不同器材
  （电工胶带 / 哑光黑塑料 / 烤漆金属）粗调。详见
  `vision/photometric._finalize_recalib`。
- [x] **K230 quirk: ulab 视角 vs image MMZ 视角 cache 不一致**：
  `image.Image.to_numpy_ref()` 文档号称"共享内存"，实测在 K230 上 ulab 端
  写入只更新到 ulab 自己那条 cache 路径，image 模块（draw_image /
  get_pixel）读 MMZ 时仍然看到旧值（`get_pixel` 全 0，但同一个 ndarray
  视图回读能看到 mixed 0/255）。
  解决：line_detector 持有 MMZ 分配的 `_roi_img`，每帧把 ALLOC_REF wrap 的
  `bin_inv_roi`（ndarray）通过 `_roi_img.copy_from(src_wrap)` 搬到 MMZ；
  OSD 用 `_roi_img` 做 draw_image source。同样的，**两步法 mask 合成
  在 K230 上也对 ALLOC_REF mask 静默无操作**——OSD 主路径必须接 MMZ
  实例。详见 `vision/line_detector.process` 中的 copy_from 调用。
- [x] **K230 ALLOC_REF 与 draw_image 兼容性总结**：
  `image.Image(alloc=ALLOC_REF, data=ndarray)` 在 K230 上**只对就地修改
  操作可靠**（`open(1)` / `binary([...])` 这类）；作为 `draw_image` 的
  **source 或 mask** 会被静默丢弃，因为 SDK 走 MMZ 物理地址而 wrap header
  没绑 MMZ。OSD 渲染端必须使用 MMZ 分配的 image 实例。
- [x] **K230 OSD 二值图 / 黑区高亮仍不显示**：即使把 `bin_inv_roi` 复制到
  MMZ image 后，`ARGB8888 OSD.draw_image(GRAYSCALE source)` 与
  `draw_image(..., mask=GRAYSCALE)` 在实板上仍可能静默无显示；右上角预览窗
  和 ROI 半透明色块都看不到。修复：`camera._draw_binary_debug` 不再走
  `draw_image` / `mask`，而是直接扫描 `detection.binary_np` 的前景连续段，
  在 OSD 上画白色二值预览与红色 ROI 高亮线段；主画面通过
  `OSD_BINARY_STRIDE_X/Y` 抽样，避免完全遮挡 VIDEO1。
- [x] **`sensor(0) snapshot chn(1) failed(3)` 崩溃**：`read_algo_frame()` 原来
  直接透传 `sensor.snapshot()` 的 RuntimeError，一次 CHN1 取帧失败就会退出；
  同时主循环里的旧 `img` 会一直持有到下一轮赋值，而 Python 会先执行右侧
  `snapshot()` 再释放旧引用，可能让 K230 的 VB buffer 紧张。修复：
  `Camera.read_algo_frame()` 捕获 snapshot 异常并按丢帧返回 `None`；
  `vision_line_tracking.py` 主循环、`photometric.bootstrap()`、camera probe /
  self_test 在用完帧后显式 `img = None`，确保下一次 snapshot 前释放旧帧。
- [x] **span renderer 性能回退（FPS≈0.4）**：全 ROI `320×150` Python 像素扫描
  与 OSD 矩形绘制过慢，一次 overlay 可耗到约 2.8 s；同时 `ulab` 单点比较
  在 `if row[x] > 0` 中不稳定，表现为右上角预览全白、ROI 内大量水平红线。
  修复：二值调试只扫描 5 条 L2 检测带（实际算法输入），并用 `int(row[x])`
  强制转 Python 标量；右上角保留 ROI 尺寸但只显示扫描带二值结果。
- [x] **span renderer 二次性能回退（FPS≈8.8）+ 红斑断续线**：上一轮把 ROI
  收缩到 5 条带后，仍是 `binary_np[y][x]` 的 ulab Python 索引在跑，每次
  ~50-100us，5 带 × 8 行 × 320 列 ≈ 12800 次 → 单次 OSD 刷新 ~770ms，algo
  FPS 被压到 8.8（实测：1 个 render 帧 770ms + 7 个普通帧 30ms = 1 秒）。
  同时 overlay 路径用 `dh=1`（fill_rows=False）只画 1 px 高横线，竖向 4 px
  采样间隙没填，肉眼看就是"断续红色线条"。修复：
  1. `bytes(binary_np)` 一次物化为 Python bytes，扫描走 ``bb[off+x]``
     字节索引（~50ns，1000× 提速），整体 OSD 刷新 <5ms；
  2. overlay 路径改 `fill_rows=True`，单采样行覆盖 `stride_y * scale_y`
     个 display 像素，红斑成片不留缝；
  3. preview / overlay 共用同一份 bytes，draw_rectangle 调用顺序合并
     在每条带的循环内，减少属性查找。
  实测预期：algo FPS 从 8.8 回到 30+ (sensor 上限)。详见
  `vision/camera._draw_binary_debug` 与 `_draw_row_spans_from_bytes`。
- [x] **二值 overlay 渲染模式拆分 + preview 独立开关**：用户希望（1）右上角
  预览能单独关闭；（2）主 ROI 红色高亮做成"完整连续半透明"，而不是只
  在 5 条带上的片状斑块。K230 `image.draw_rectangle` 签名只接受
  `(x,y,w,h,color,thickness,fill)`——**不支持 alpha，也不支持 4 元组
  RGBA**（见 `docs/k230_canmv_docs/api/openmv/image.md`）。硬件 OSD 的
  ARGB8888 真 alpha 仅能改 buffer 字节，但前述 ulab↔image cache quirk 让
  这条路风险高。改为：
  1. 加 `DEBUG_SHOW_BINARY_PREVIEW`（默认 True）与 `DEBUG_SHOW_BINARY`
     彻底分离；preview / overlay 各自独立判定；
  2. 加 `OSD_BINARY_OVERLAY_MODE`，最初实现 3 档（`bands_only` /
     `full_solid` / `full_dither`），后扩展为 5 档（见下条）；
  3. 全 ROI 扫描共 320×150=48000 次字节索引 ≈ 2.4ms；前景率 ~1-3%
     场景下 draw_rect 调用 ~300-1000 次，OSD 总耗时 < 30ms。
  详见 `vision/camera._draw_overlay_full_dither` 与 `_draw_overlay_bands`。
- [x] **桌面调试 V=0/5 → 加 `LINE_DETECTION_PROFILE` 双档**：用户对桌面
  白纸上的黑色线缆做模拟时，红色 overlay 显示正确（L0/L1/L2 主干通畅），
  但 V=0/5 全程 hold/lost。日志反推：`band_fg=89/12800 (0.7%)`，平均每带
  18 px → mass ≈ 4500，远低于 plan §6.2 默认 `MIN_MASS_PER_BAND[i]≥4000~10000`
  阈值；线缆圆柱中间反光只剩两条 1-2 px 边线，width 也撞 `W_MIN[i]=5..12`。
  这正是 task_log §4 早期 TODO 提示的"装车后实测重定"场景，桌面前后不切档
  必然 V=0。修复：
  1. config 加 `LINE_DETECTION_PROFILE` 字符串开关，"bench" / "track" 两档；
  2. `_MIN_MASS_PER_BAND_BENCH=(300,400,500,700,900)`、
     `_W_MIN_PX_PER_BAND_BENCH=(1,1,1,1,1)`、
     `_W_MAX_PX_PER_BAND_BENCH=(320,320,320,320,320)`、
     `_COL_SUM_THR_FOR_WIDTH_BENCH=0`、
     `DELTA_CX_MAX_PX=320`、`Q_L2_MASS_NOMINAL_TOTAL=10000`；
     全部按 0.7%~4% 前景率与任意摆放线缆反推，让有信号的带过、没信号的带挂；
  3. track 一组保持 plan §6.2 装车预期值不动，避免回赛道时反复改阈值；
  4. `vision_line_tracking.main()` 启动日志多打一行 `L2 thresholds: ...`，
     方便实测时确认当前 profile 已生效。
  bench → track 切回时只改这一行字符串，不改其他参数。详见 `config.py`
  顶部 ``LINE_DETECTION_PROFILE`` 区块。
- [x] **删除 line_detector copy_from MMZ 死路径**：camera.py OSD 改走
  `bytes(binary_np)` 字节扫描后，``detection.binary_image``（MMZ 镜像）
  零消费，但 line_detector 每帧仍 `_roi_img.copy_from(src_wrap)` 做
  ~48 KB memcpy ≈ 1-2 ms。删除后：
  1. `_roi_img = image.Image(GRAYSCALE)` 构造期 MMZ 分配去掉，常驻内存
     省一份；
  2. process() 末尾 copy_from 整段删掉，主路径每帧省 1-2 ms（实测预期
     L0+L1+L2 从 ~10-13ms 降到 ~9-11ms）；
  3. L1 关闭时 ALLOC_REF wrap 也不再创建（之前是无条件创建供 copy_from
     用），又省一次 image.Image 构造。
  ``DetectionResult.binary_image`` 字段保留为 None，向后兼容。
  详见 `vision/line_detector.process` / `DetectionResult` docstring。
- [x] **binary overlay 独立刷新频率 + dither 多档密度**：1Hz overlay 看不出
  实时变化（红斑跟不上线缆 / 镜头抖动），用户希望可调到每帧；同时希望
  dither 提供更稀疏档以便每帧时降低 CPU 压力。改为：
  1. 加 `OSD_BINARY_REFRESH_MS`（默认 0=每帧，正数=ms 节流）；
     `Camera.maybe_update_binary(now)` 与 `maybe_update_fps` 并列；
  2. 主循环拆两路触发：`maybe_update_fps`（1Hz）只用来更新 `cached_lines`
     文字内容；`maybe_update_binary`（每帧 / 用户设定）触发 render_overlay
     用 `cached_lines` + 最新 `detection` 整体重画，文字行高频时不会抖动；
  3. dither 扩成 3 档：
     - `full_dither_50`：(x+y)%2==0 棋盘格，50% 红色密度（旧默认）；
     - `full_dither_25`：偶行 × 偶列，25% 密度，OSD 矩形数减半；
     - `full_dither_12`：偶行 × 4 取 1 列，列起点交错避免竖排红线，
       12.5% 密度，最稀疏，每帧刷新仍 < 10ms；
  4. 兼容旧名 `full_dither` → 自动映射为 `full_dither_50`。
  详见 `vision/camera.maybe_update_binary` 与 `_draw_overlay_full_dither`。
- [x] **IDE 停止信号被调试容错吞掉**：K230/CanMV IDE 的停止请求有时以
  `IDE interrupt` 异常从 `snapshot()` / OSD 绘图 / 图像统计 API 抛出；早期
  为了容错写的 `except Exception` 会把它当普通错误打印后继续循环，导致程序
  无法终止。修复：新增 `vision.interrupts.reraise_if_stop()`，在 camera /
  line_detector / photometric 的宽泛异常处理里遇到 `KeyboardInterrupt` 或
  `IDE interrupt` 立即重新抛出，让主入口进入 `finally` 正常释放资源。
- [x] **外接按键控制主 ROI 红色 overlay**：调试时需要在不重启程序、不改
  `config.py` 的情况下开关 `DEBUG_SHOW_BINARY` 对应的主画面红色二值叠加。
  选用开放引脚表中的 Header NO.9 / `IO_42`：按键一端接 `IO_42`，另一端接
  `GND`，软件启用 `Pin.PULL_UP`，未按下为高电平、按下为低电平。实现：
  1. `config.py` 增加 `DEBUG_BINARY_BUTTON_*` 配置，默认 `IO_42`、低电平
     有效、80 ms 去抖；
  2. 新增 `vision/gpio_button.py`，封装 `FPIOA.set_function(IO, FPIOA.GPIOx)`
     与 `Pin(IO, Pin.IN, pull=Pin.PULL_UP)`，主循环轮询并只在稳定按下沿返回
     一次事件，长按不重复触发；
  3. `Camera` 增加 `binary_overlay_enabled()` /
     `set_binary_overlay_enabled()`，运行期更新已缓存的 `_binary_overlay_enabled`；
  4. `vision_line_tracking.py` 每帧轮询按键，触发后立即
     `render_overlay(cached_lines, detection=detection)`，并在 OSD 文字行显示
     `BIN ON/OFF`。本地 linter 与 Python AST 语法检查通过；板端还需实测按键
     翻转和去抖效果。
- [ ] **OSD 信息密度**：阶段 B 加了 Q/V/cxN/thr 一行，逼近 3 行 OSD 上限；
  阶段 C 起 IPM 后会再加 `e_y_mm / ψ_e_mrad / R̂_mm`，需要决定是否拆
  `debug_overlay.py` 单独模块。

---

## 5. 进入下一阶段的前置条件

**代码侧已可进入阶段 C**（IPM + 路径误差生成）。下列实测项不阻塞代码推进，
但**完成赛道实物前**必须补齐：

1. §3 验收表 6 行全部回填实测数据；
2. 至少 3 段视频（L1=image_open / L1=none / 阴影场景），存到 `tests/golden/`
   作为阶段 C 的回归对照；
3. 把实测后的 `MIN_MASS_PER_BAND` / `W_MIN/MAX_PX_PER_BAND` /
   `Q_L2_MASS_NOMINAL_TOTAL` 写回 `config.py` 并 commit，CONFIG_VERSION
   保持 `phaseB-0.1` 不变（不破坏接口，只调参）。

---

## 6. 给阶段 C 的接力条目

- **算法预算复核**：阶段 A v2.1 修订给后续阶段 ~17 ms / 帧；阶段 B 实测后
  应给出"L0 / L1(image_open) / L2(5 带) / Q / OSD / 反相 / wrap"各项耗时
  分解，让阶段 C 知道还能塞多少 IPM + RANSAC（plan §6.1 表 L3a 预算 8 ms）。
- **DetectionResult 接口**：阶段 C 的 `ground_mapper` 直接消费
  `DetectionResult.bands[i].cx_px / y_top / y_bot`，不依赖 binary_np；
  若阶段 C 要在 IPM 后做"地面坐标的 RANSAC"，可以读 `cx_near_px / cx_far_px`
  做 sanity check，IPM LUT 把每带 cx 直接换算成 (x_g, y_g)。
- **Q 升级路径**：阶段 C 的 `compute_q` 会在 quality.py 里新增 `compute_q_full`，
  叠加 IPM RANSAC 内点率 + R̂ 先验约束；当前 `compute_q_l2` 保留为子项不动。
- **photometric 与 calib.json 的合并**：阶段 C 实现 `config.load_calibration`
  时，新增 photometric 节（schema 与 `tools/calibrate_photometric.py` 当前
  payload 字段一致），主入口启动期改为"先尝试加载 calib.json，否则跑
  bootstrap 兜底"。
- **L1 后端最终选型**：装车实测后定 `LINE_L1_BACKEND` 默认值（image_open
  vs none），写进 plan errata，避免阶段 C 起继续"两套都跑"。

---

## 7. 日志与 OSD 字段含义速查

### 7.1 主循环每 ~5 秒一行的 `[VLT] algo_fps=...` 日志

来自 `vision_line_tracking.py` 主循环（按 `LOG_INTERVAL_MS` 节流，默认 5000ms）。
单行示例：

```
[VLT] algo_fps=30.6 period=32.7ms frames=181 Q=80.0(good) V=0/5 cxN=-1.0 cxF=-1.0 thr=51 mu=88.5 sig=17.1 mem=3965696 (min=3950208 max=3966272 drift=0.4%)
```

| 字段 | 单位 | 含义 | 健康范围 |
|---|---|---|---|
| `algo_fps` | Hz | 算法链路（CHN1 GRAYSCALE → detector）的实测帧率，5 s 窗口平均 | 30~33（sensor 上限） |
| `period` | ms | `1000 / algo_fps`，单帧周期 | < 33（≈ 30 FPS） |
| `frames` | 帧 | 自启动累计算法帧数（不含丢帧） | 单调递增，无回退 |
| `Q` | 0~100 | `Q_L2` 评分（mass + 连续性 + 有效带占比 三项加权，见 §6 / `vision/quality.py`） | 80+ 为 good |
| `(<grade>)` | 标签 | `Q` 落点：`good ≥80 ≥ degrade ≥60 ≥ hold ≥40 ≥ lost` | good |
| `V` | n/N | 5 条扫描带通过硬约束的数量 / 总数（`n_valid` / `BAND_COUNT`） | 4/5 ~ 5/5 |
| `cxN` | px | **近带**（bands[-1]，y 最大）`cx_px`，黑线在算法分辨率下的横向中心列；`-1.0` = 该带无效 | `[0, ALGO_WIDTH)` 内 |
| `cxF` | px | **远带**（bands[0]，y 最小）`cx_px`；含义同 `cxN` | 同上 |
| `thr` | 灰度 | `photometric.threshold`，本帧 L0 二值化阈值 | bench: 50~80；track: 60~100 |
| `mu` | 灰度 | 上次 bootstrap / 漂移重标定测得的 ROI 背景均值 `μ_bg` | 取决于环境光 |
| `sig` | 灰度 | 同上的 `σ_bg`（背景标准差） | < μ/3 才算单峰高斯 |
| `mem` | byte | `gc.mem_free()` 当前剩余堆 | 略波动；震荡 < 10% 为 OK |
| `min` | byte | 启动以来 `mem_free` 的最小观察值 | 关注 drift% |
| `max` | byte | 启动以来 `mem_free` 的最大观察值 | 同上 |
| `drift` | % | `(max-min)/max` 内存震荡幅度，plan §12 阶段 A 验收要求 ≤ 10% | < 5% 为优 |

附加可能出现的告警字段（满足触发条件才会打印）：

- `[photometric] drift trigger mu_bg X -> Y (delta=Z)`：ROI 均值漂移 ≥
  `PHOTO_DRIFT_TRIG_DELTA_MU`，启动 30 帧重标定；
- `[photometric] drift recalib done frames=N mu_bg=... ...`：重标定收尾；
- `[photometric] fallback rejected (mu=... sigma=... thr_fb=...) -> use Otsu only`：
  `μ−kσ` 落到病态区（≤0 或 ≥ Otsu），降级为单 Otsu；
- `[VLT] snapshot failed N times in a row`：连续丢帧 ≥ 10 次。

### 7.2 OSD（屏幕叠加）三行文本

`render_overlay()` 在 1 Hz 触发时刷新；与 §7.1 同源数据。

| OSD 行 | 模板 | 字段说明 |
|---|---|---|
| 1 | `FPS xx.x  (T xx.x ms)` | 同 `algo_fps` / `period`；`T > FRAME_PERIOD_ALERT_MS` 时整行红 |
| 2 | `Q xx.x  V n/N  cxN xx.x  thr nn` | 同上 4 字段；`Q < Q_HOLD` 时整行红 |
| 3a (条件) | `PHOTO recal n/30` | 仅在 photometric 重标定中显示（红） |
| 3b (条件) | `MEM xxx KB  (drift x.x%)` | 仅在 `drift% ≥ MEM_DRIFT_ALERT_PCT` 或 `mem < MEM_LOW_ALERT_BYTES` 时显示（红） |
| 3c (条件) | `CAP n/N` | 仅在 `CAPTURE_ENABLE=True` 采样模式下显示 |

### 7.3 `[camera.dbg] binary` 调试行（启动后前 ~10 帧打印）

```
[camera.dbg] binary thr=53 band_fg=80/12800 (0.6%) mode=full_dither_12 preview=0 overlay=247
```

| 字段 | 含义 |
|---|---|
| `thr` | 当前 L0 阈值（同上 `thr`） |
| `band_fg / total` | 5 条带共 `BAND_COUNT × BAND_HEIGHT_PX × ROI_W` 像素中前景数（`mass_total / 255`） |
| `(x.x%)` | 前景占比；装车后正常黑线场景应 1~5% |
| `mode` | `OSD_BINARY_OVERLAY_MODE` 实际生效值（`bands_only` / `full_solid` / `full_dither_50/25/12`） |
| `preview` | 右上角预览窗本帧画的 OSD 矩形数（`DEBUG_SHOW_BINARY_PREVIEW=False` 时恒为 0） |
| `overlay` | 主画面 ROI 红色高亮本帧画的 OSD 矩形数 |

### 7.4 启动期一次性日志

```
[VLT] vision_line_tracking start, config=phaseB-0.1, debug=True, profile=bench
[VLT] L2 thresholds: MIN_MASS=(...) W=[(...)..(...)] COL_THR=0 Δcx_max=60
[camera] binary overlay setup: overlay=True preview=False mode=full_dither_12 ...
[camera] request: sensor=1280x720@60, display→CHN0=YUV420SP, algo→CHN1=GRAYSCALE
[camera] CHN0 (display): 800x480
[camera] CHN1 (algo   ): 320x240
[photometric] bootstrap done frames=30 mu_bg=... sigma_bg=... thr_otsu=... threshold=...
```

| 字段 | 含义 |
|---|---|
| `config=phaseB-0.1` | `CONFIG_VERSION`，配置 schema 版本号 |
| `profile=bench/track` | `LINE_DETECTION_PROFILE`：硬约束阈值组（见 §4 桌面 vs 装车切档说明） |
| `MIN_MASS / W / COL_THR / Δcx_max` | 当前 profile 实际生效的 L2 硬约束值 |
| `binary overlay setup` | OSD 二值调试初始化：是否启用、模式、ROI 几何参数 |
| `display→CHN0 / algo→CHN1` | 双通道格式：`OSD_PIXFORMAT` / `ALGO_PIXFORMAT` |
| `bootstrap done` | photometric 30 帧自适应初标定结果（`μ_bg / σ_bg / Otsu / 最终 threshold`） |
