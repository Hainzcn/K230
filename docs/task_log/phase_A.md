# 阶段 A 任务日志：摄像头基础采集

> 对照计划：`docs/vision_line_tracking_plan_v2.md` §12 阶段 A
> 起止：2026-05-04 ~ ___（待回填）___
> 负责人：___
> 当前状态：**代码侧已完成；硬件实测/采样/帧率验收待补充**

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

---

## 5. 进入下一阶段的前置条件

进入阶段 B 之前必须满足：

1. 本日志 §3 表格的 4 行测试全部填写并打勾。
2. `docs/hardware_profile.md` §2 / §3 实测字段全部回填。
3. 至少录制 1 段 30 秒稳态运行视频（可由 IDE 屏幕录制完成）。
4. 至少 100 张静态赛道样张落盘 `/sdcard/captures/`，并人工抽检 10 张确认黑线
   清晰可辨、未严重过曝/欠曝。
