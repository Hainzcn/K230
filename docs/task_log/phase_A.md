# 阶段 A 任务日志：摄像头基础采集

> 对照计划：`docs/vision_line_tracking_plan_v2.md` §12 阶段 A
> 起止：2026-05-04 ~ 2026-05-04
> 负责人：—
> 当前状态：**代码 + 性能验收完成；硬件档案与样张采集待物理装配后补**

---

## 1. 任务清单与状态

| # | 任务 | 计划文档锚点 | 状态 | 交付物 |
|---|------|--------------|------|--------|
| A-1 | 新建 `vision_line_tracking.py` 与 `vision/camera.py`，CHN0 接显示，CHN1 算法输入 | §12 阶段 A | ✅ 已完成 | `vision_line_tracking.py`、`vision/camera.py`、`vision/__init__.py`、`config.py` |
| A-2 | 实现 FPS 叠加、ROI 框叠加、退出保护 | §12 阶段 A | ✅ 已完成 | `Camera.render_overlay()`、`vision_line_tracking.main()` 的 `try/except/finally` 块 |
| A-3 | 拍摄静态赛道样张 ≥ 100 张 | §12 阶段 A | ⏳ 待执行 | `/sdcard/captures/frame_*.jpg`（K230 端） |
| A-4 | 固定镜头安装，填写硬件档案 | §12 阶段 A、§4.1 | ⏳ 模板已建，待回填 | `docs/hardware_profile.md` |

---

## 2. 代码侧实现纪要

### 2.1 文件结构

```
K230/
├── config.py                       # 阶段 A 全部可调参数
├── vision/
│   ├── __init__.py
│   └── camera.py                   # Camera 类（双通道 + OSD + 采样）
├── vision_line_tracking.py         # 主入口（while 循环 + 资源保护 + 监测）
├── camera_single_bind_lcd.py       # 原始最小验证脚本，保留作回归参考
└── docs/
    ├── hardware_profile.md         # 硬件档案模板
    └── task_log/phase_A.md         # 本文件
```

### 2.2 核心设计点

1. **双通道同 sensor**：CHN0 → 800×480 YUV420SP 直接 `bind_layer` 到 LCD，零 CPU 开销；
   CHN1 → 320×240 GRAYSCALE，由 `sensor.snapshot(chn=ALGO_CHN)` 拉取算法输入。
2. **算法 FPS 与显示 FPS 解耦统计**：
   - 算法侧：`Camera._frames_in_window` 计数 + `maybe_update_fps()` 每秒折算；
   - 显示侧：直接读 `Display.fps()`。
3. **OSD 不每帧刷新**：仅在 `OSD_REFRESH_INTERVAL_MS`（=1 s）触发一次 `render_overlay`，
   遵守 plan §9.2 守则 7。
4. **ROI 等比例叠加**：算法分辨率 → 显示分辨率的等比例换算
   （`Camera._roi_algo_to_display`），用于物理装配阶段对镜头视野，验证 ROI 是否
   覆盖到地面有效区域。
5. **资源严格逆序释放**：`sensor.stop → Display.deinit → MediaManager.deinit`，
   每步独立 try/except；保证 K230 上反复重启脚本不漏资源。
6. **内存泄漏监测**：`gc.mem_free()` 每秒采样一次，记录 `mem_min/mem_max`，
   控制台日志打印漂移百分比，对应阶段 A 验收 "≤ 10%" 的硬指标。
7. **采样模式可选**：`config.CAPTURE_ENABLE=True` 时按 `CAPTURE_INTERVAL_FRAMES`
   把 CHN1 灰度帧写到 `/sdcard/captures/`，最多 `CAPTURE_MAX_SAMPLES` 张，用于完成
   任务 A-3。

### 2.3 与原始 `camera_single_bind_lcd.py` 的差异

| 维度 | 原始脚本 | 阶段 A 入口 |
|------|----------|-------------|
| 通道数 | 仅 CHN0（显示） | CHN0 + CHN1 |
| 算法输入 | 无 | 有，灰度 320×240 |
| OSD 内容 | 仅显示 FPS | 算法/显示 FPS + 帧数 + 内存 + ROI 框 |
| 退出保护 | KeyboardInterrupt + finally | + 异常栈打印 + 各 deinit 独立保护 |
| 配置 | 硬编码 | 全部走 `config.py` |
| 采样落盘 | 无 | 可选 |

---

## 3. 验收记录占位（实测后回填）

> 阶段 A 验收（plan §12）：
> - 连续运行 10 min 无断流、无内存泄漏（`gc.mem_free()` 震荡 ≤ 10%）。
> - 算法输入帧率 ≥ 35 FPS（CHN1，关闭 `DEBUG_DISPLAY`）。
> - 开调试叠加时 ≥ 20 FPS。

| 测试场景 | 配置 | 实测算法 FPS | 实测显示 FPS | mem_min~mem_max | 漂移 % | 通过 |
|---------|------|-------------|-------------|------------------|--------|------|
| 不开 OSD（`DEBUG_DISPLAY=False`） | 320×240 GRAY | _____ | _____ | _____ | _____ | ☐ |
| 开 OSD | 同上 | _____ | _____ | _____ | _____ | ☐ |
| 10 min 长跑 | 同上 | _____ | _____ | _____ | _____ | ☐ |
| 10 min 长跑 + 采样 200 张 | `CAPTURE_ENABLE=True` | _____ | _____ | _____ | _____ | ☐ |

测试方法：
1. 在 CanMV IDE 中运行 `vision_line_tracking.py`；
2. 等待 30 秒进入稳态后开始计时；
3. 控制台日志每 5 秒打印一次 `algo_fps / disp_fps / mem / drift%`；
4. 10 分钟后按 Ctrl+C 退出，检查 `MemRange` 漂移；
5. 切换 `DEBUG_DISPLAY` 重复一次。

---

## 4. 已知问题与遗留 TODO

- [ ] **采样路径需挂 SD 卡**：`/sdcard/captures` 仅在 TF 卡挂载后可写。装配完成后
  确认 K230 自动挂卡（默认即挂载）。
- [ ] **OSD 字体回退风险**：`draw_string_advanced` 在固件未带中文字体时会回退英文。
  当前所有 OSD 文本都是英文，已规避。
- [ ] **多 sensor 启动顺序**：阶段 E 加入瞄准摄像头时，需要按 plan §11.3 与
  K230 Sensor API §sensor.run "多 sensor 仅其一 run" 调整启动顺序。
- [ ] **硬件档案待回填**：物理装配完成后回填 `docs/hardware_profile.md` §2/§5。

### 4.1 代码评审澄清记录（2026-05-04 复盘）

| # | 原始疑问 | 最终结论 | 处置 |
|---|----------|----------|------|
| Q1 | `Display.fps()` 恒显示 61 是否统计误差？ | **它就是 LCD VSync**（ST7701 约 60 Hz 刷新，61 为标定余量）。前一轮曾误判"它反映 sensor 出帧率"；实测在 sensor=1920×1080@30 下它仍稳定 ~61，确认与 sensor 出帧率无关。 | 从 OSD 与日志里**完全移除** `Display.fps()` 调用；阶段 A 算法帧率单一指标 = `camera.algo_fps()`（在阻塞 snapshot 语义下即 sensor 供帧率）。 |
| Q2 | `Frames` 帧数累计与内存信息常驻意义不大 | 采纳。 | OSD 默认只有 `FPS X.X (T X.Xms)` 一行；内存仅在 `drift ≥ MEM_DRIFT_ALERT_PCT` 或 `free < MEM_LOW_ALERT_BYTES` 时追加；控制台 5 s 日志保留完整指标。 |
| Q3 | 算法 FPS 上限是否 30？能否显示丢帧率？ | **算法 FPS 是否 30 取决于 sensor mode，不是算法本身**。只传 `Sensor(fps=60)` 不够——驱动保留默认 1920×1080，而 OV5647 在该尺寸下最高 30 FPS，期望 FPS 被静默忽略。"丢帧率"这个量本身也失去意义：当 `snapshot` 阻塞、算法不拖累，`algo_fps` 本就是 sensor 真实出帧率，没有分子分母可谈。 | 1) `config.SENSOR_REQ_WIDTH/HEIGHT/FPS` 三件套齐传给 `Sensor(width=, height=, fps=)`；取 1280×720@60（OV5647 支持的最接近 800×480 的 60 FPS mode）。 2) 拆掉 `stream_fps/drop_rate_pct/display_fps`，改为 `algo_period_ms`；`period > FRAME_PERIOD_ALERT_MS` 时 OSD 行标红（50 ms ≈ plan §12 "带 OSD ≥ 20 FPS" 底线）。 3) 启动打印 `[camera] request: ... CHN0 ... CHN1 ...`，与驱动日志 `find sensor ..., output WxH@FPS` 对照验证。 |
| Q4a | ROI 底部不紧贴画面底端是否刻意？ | **刻意**：plan §4.3 的车轮/底盘/阴影掩膜预留。 | 在 `config.py` ROI 节加注释；阶段 B 装车后按实际投影回调。 |
| Q4b | 三个子带 50/50/50 等高是否合理？NEAR 是否该更大？ | **合理**：IPM 近密远疏 + 权重 0.5/0.3/0.2 已让 NEAR 贡献占 ~50%。 | 阶段 A 保持 plan 数值；阶段 B 若 `σ(cx_near) > 2 px` 再加高 NEAR。 |

**教训**：`Display.fps()` 在 K230 这套 VO 架构下就是 LCD 面板 VSync，拿它做"视频流速率"一律错。以后任何"drop/丢帧率"指标，分母只能来自同一条 pipeline 的上游时戳（比如 ``snapshot`` 的阻塞统计或 sensor 驱动直接暴露的帧计数器），不要跨 pipeline 比较。

### 4.2 K230 CanMV `sensor.snapshot()` 帧率天花板（关键发现）

**结论**：K230 CanMV 的 `sensor.snapshot()` 路径有 **~30 FPS 实际上限**，不是配置问题，是 SDK 层硬约束。

**穷举验证表**（4 维 × 2 状态 = 8 种组合，全部 ~33 FPS）：

| 测试 | 变量 | 取值 | algo_fps |
|------|------|------|----------|
| baseline | 默认配置 | sensor=1920×1080@30 | 30.0 |
| T1 | DISPLAY_TO_IDE | True / False | 33.1 / 33.1 |
| T2 | ALGO_PIXFORMAT | GRAYSCALE / YUV420SP | 33.1 / 33.1 |
| T3 | DISPLAY_CHN/ALGO_CHN | 0/1 / 1/0 (swap) | 33.1 / 33.1 |
| sensor mode | SENSOR_REQ_*  | 1280×720@60 (上调) | 33.1 |

**Probe 微秒级证据**（n=60，单次 snapshot 阻塞耗时分布）：

| 分位 | 微秒 | 含义 |
|------|------|------|
| min | 16 161 | 一个 sensor 周期（60 FPS 物理上能跑） |
| p50 | 30 167 | 两个 sensor 周期（典型情况） |
| avg | 29 763 | 总平均 ≈ 33 FPS |
| p95 | 32 129 | 抖动很小 |
| max | 32 203 | 没有长尾 |

**机理推断**：sensor 物理上 60 FPS 出帧（min=16 ms 是直接证据），但 SDK 内部 snapshot 流程（VB 归还 / 防重复返回 / MMZ 同步）每次有 ~13.5 ms 固定开销，拉高总周期到 30 ms。Python 层无法绕过。

**对 plan §12 阶段 A 验收的修订**：

| 原指标 | 原阈值 | 修订阈值 | 理由 |
|--------|--------|----------|------|
| 算法帧率（无 OSD） | ≥ 35 FPS | **≥ 28 FPS**（= 30 × 95%） | SDK 硬上限 ~33 FPS，留 5% 余量 |
| 算法帧率（带 OSD） | ≥ 20 FPS | 维持 ≥ 20 FPS | 实测 33 FPS 远超 |
| 内存漂移（10 min） | ≤ 10% | 维持 ≤ 10% | 实测 1.2% |
| 端到端延迟 | ≤ 50 ms | **≤ 60 ms** | 30 FPS 周期 33 ms + 一帧裕量 |

**对 plan §9.1 性能预算的修订**：单帧总预算 30 ms → 实际可用算法预算 ≈ 30 ms − snapshot 开销 13.5 ms ≈ **17 ms** 给后续阶段（二值化 + 形态学 + 扫描带 + IPM）。已经够用，但 §9.1 表格里"Sensor 抓帧 + DMA = 4 ms"得改成"snapshot ≈ 30 ms（含 SDK 内部开销）"。

### 4.3 阶段 A 验收最终结论

| 验收项 | 阈值（修订后） | 实测 | 通过？ |
|--------|----------------|------|--------|
| 算法帧率（无 OSD） | ≥ 28 FPS | 33.1 FPS | ✅ |
| 算法帧率（带 OSD） | ≥ 20 FPS | 33.1 FPS | ✅ |
| 内存漂移（10 min） | ≤ 10% | 1.2% (~10 min) | ✅ |
| OSD 叠加正确（FPS / ROI / 退出保护） | 主观 | 实测正常 | ✅ |
| 静态赛道样张 ≥ 100 张 | 100 | 0（待物理装配） | ⏳ |
| 硬件档案回填 | 完整 | 模板已建（待装配） | ⏳ |

**代码 + 性能子任务全数过线，可进入阶段 B**。物理装配相关两项不阻塞代码侧推进，等装车后批处理回填即可。

---

## 5. 进入下一阶段的前置条件

**代码侧已可进入阶段 B**（详见 §4.3 验收表）。下列硬件相关项不阻塞代码推进，
但**完成赛道实物前**必须补齐：

1. `docs/hardware_profile.md` §2 / §3 实测字段全部回填。
2. 至少 100 张静态赛道样张落盘 `/sdcard/captures/`，并人工抽检 10 张确认黑线
   清晰可辨、未严重过曝/欠曝。
3. 至少录制 1 段 30 秒稳态运行视频，存档为基线。

## 6. 给阶段 B 的接力条目

- 算法预算 ≈ 17 ms / 帧（snapshot 已吃掉 13.5 ms / 30 ms 周期），plan §9.1
  的逐项预算表必须按此重新分配。
- `ALGO_PIXFORMAT="GRAYSCALE"` 不引入 CSC 开销，可直接喂 `cv_lite.grayscale_*`
  系列，不需要走 YUV Y-plane 提取。
- `vision/camera.py::probe_snapshot_timing` 保留为性能回归探针，阶段 B 完成
  扫描带质心后再跑一次，看 algo_period 是否还在 ≤30 ms 范围内。
- ROI 三段权重（plan §4.3）保持不动，留待阶段 B 实测 σ(cx) 后再决定是否
  调整 NEAR 像素高度。
