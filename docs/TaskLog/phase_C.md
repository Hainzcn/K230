# 阶段 C 任务日志：IPM 与路径误差生成

> 对照计划：`docs/vision_line_tracking_plan_v2.md` §12 阶段 C
> 起止：2026-05-07 ~
> 负责人：—
> 当前状态：**代码完成（CONFIG_VERSION=phaseC-0.1，含 IPM 单应 / RANSAC 圆 +
> LSQ 直线 fallback / EMA + 符号防抖 / Q_full / OSD 几何叠加 / 主入口装配）；
> PC 端单测全部通过；装车实测 + 4 项验收硬指标待回填**

---

## 1. 任务清单与状态

| # | 任务 | 计划文档锚点 | 状态 | 交付物 |
|---|------|--------------|------|--------|
| C-1 | §5.2 四点标定 + 生成 IPM LUT | §5.2 / §11.1 | ✅ PC 端落地（K230 端打靶工具登记 TODO） | `tools/calibrate_ipm.py`（DLT 解 H + 校验 + 写 calib.json） |
| C-2 | RANSAC 圆弧 (L3a) + 一阶回归 (L3b) | §6.3 | ✅ 已完成 | `vision/geometry.py`：`fit_circle_3pt` / `fit_circle_ransac` / `fit_line_lsq` / `compute_path_errors_*` |
| C-3 | EMA 估计 `e_y` / `ψ_e` | §7.4 | ✅ 已完成 | `vision/estimator.py`：`EmaEstimator` / `SignDebounce` / `PathErrorEstimator` |
| C-4 | OSD：IPM 中心线 / 圆心 / 切线箭头 | §12 阶段 C | ✅ 已完成 | `vision/debug_overlay.py::PathOverlayInfo` + `_draw_path_geometry` |
| C-5 | IPM 单应映射（plan §5.2 LUT 替代） | §5.2 | ✅ 已完成 | `vision/ground_mapper.py`：`GroundMapper`（5 点 H 矩阵展开 + 解析占位 H） |
| C-6 | Q_full 评分（plan §6.6 全权重） | §6.6 | ✅ 已完成 | `vision/quality.py::compute_q_full`，与 `compute_q_l2` 共存 |
| C-7 | 主入口装配 + 5s 日志扩展 | §11.3 / §13.1 | ✅ 已完成 | `vision_line_tracking.py` |
| C-8 | 装车后侧拉 ±30 mm `e_y` 误差 ≤ ±5 mm | §12 阶段 C 验收 | ⏳ 待装车 | — |
| C-9 | 装车后旋转 ±10° `ψ_e` 误差 ≤ ±0.03 rad | §12 阶段 C 验收 | ⏳ 待装车 | — |
| C-10 | 推车沿黑线 `e_y` 符号稳定不跳变 | §12 阶段 C 验收 | ⏳ 待装车 | — |
| C-11 | 端到端延迟 ≤ 60 ms（v2.1 errata） | §12 阶段 C 验收 / §18.4 | ⏳ 待装车 | — |

---

## 2. 代码侧实现纪要

### 2.1 文件结构（与计划书 §11.1 一致）

```
K230/
├── config.py                          # CONFIG_VERSION=phaseC-0.1，新增 IPM / RANSAC / EMA / Q_full
├── tools/
│   ├── calibrate_photometric.py       # 阶段 B
│   └── calibrate_ipm.py               # 阶段 C：PC 端 DLT 解 H 写 calib.json
├── vision/
│   ├── __init__.py                    # __all__ 加 ground_mapper / geometry / estimator
│   ├── camera.py                      # 阶段 A；render_overlay 新增 path 透传
│   ├── debug_overlay.py               # 阶段 B + 阶段 C：PathOverlayInfo + _draw_path_geometry
│   ├── photometric.py                 # 阶段 B
│   ├── line_detector.py               # 阶段 B
│   ├── quality.py                     # 阶段 B compute_q_l2 + 阶段 C compute_q_full
│   ├── ground_mapper.py               # 阶段 C：IPM 单应映射 + 解析占位 H
│   ├── geometry.py                    # 阶段 C：RANSAC 圆 / LSQ 直线 / e_y ψ_e 公式
│   └── estimator.py                   # 阶段 C：EMA + 符号防抖 + PathErrorEstimator
├── vision_line_tracking.py            # 装配 IPM/RANSAC/EMA/Q_full + OSD 文本扩展 + 5s 日志
└── docs/task_log/phase_C.md           # 本文件
```

### 2.2 数据流（阶段 C 在阶段 B 之后追加的部分）

```text
detection (阶段 B)
  └─► GroundMapper.bands_to_ground       # 5 条带 cx → 地面坐标 mm
        └─► [(x_g, y_g) × valid bands]
              └─► fit_circle_ransac      # plan §6.3 L3a：枚举 C(N,3) + 半径先验
                    ├─► succ → compute_path_errors_arc → (e_y, psi_e)
                    └─► fail → fit_line_lsq → compute_path_errors_line → (e_y, psi_e)  # plan §6.3 L3b
                          └─► PathErrorEstimator.update                # EMA + 符号防抖
                                └─► (e_y_filt_mm, psi_e_filt_mrad)
                                      └─► compute_q_full(detection, arc)   # plan §6.6 全权重
                                            └─► PathOverlayInfo
                                                  └─► debug_overlay._draw_path_geometry
                                                        ├─► 5 点 cx 折线（绿）
                                                        ├─► 圆心反投点 / ROI 边缘箭头（品红）
                                                        └─► 近带切线箭头（橙）
```

### 2.3 核心设计点

#### 2.3.1 IPM 单应映射：5 点直接展开（不构建 ROI 全像素 LUT）

plan §5.2 给的两种实现：(1) `img.rotation_corr(corners=...)` 一行代码；
(2) 预计算 ROI 全像素 LUT。我们走"第三条路"：detector 每帧只产出 5 个候选
点（5 条扫描带的 cx），ROI 全像素 LUT 是浪费。直接做 9 元素 H 矩阵展开
（9 mul + 6 add + 2 div / 点），每帧 5 点 ≈ 0.05 ms，比上面两种实现都更省，
且零启动成本（无 LUT 构建）。

`GroundMapper.bands_to_ground(bands)` 把每条带的 `(cx_px, mid_y_px)` 喂给
H：`(u, v) = (cx_px, (y_top + y_bot) / 2)`。无效带占位返回 `valid=False`。

#### 2.3.2 三档加载状态（`mode`）

- `"calibrated"`：`calib.json` 里有有效 `ipm.H_3x3`（9 个浮点 + image_wh
  匹配 320×240）。装车后稳态。
- `"default"`：calib 缺失但 plan §4.1 安装几何（`MOUNT_H_CAM_MM` /
  `MOUNT_PITCH_DEG` / `SENSOR_HFOV_DEG`）能解析推导一份占位 H。OSD 显示
  琥珀色 `CALIB:DEFAULT (estimate, run calibrate_ipm.py)`，提醒用户
  `e_y` / `ψ_e` 数值有几十 mm 系统偏差。桌面 bench 主要走这条路径。
- `"none"`：连解析推导都失败（极少见，配置参数被改坏才会触发）。OSD
  显示红色 `NO CALIB`，主循环跳过 IPM/RANSAC/EMA 链路。阶段 B 流水线
  （光度 + L2 + Q_L2 + 二值 overlay）继续正常工作。

#### 2.3.3 占位 H 的解析推导（plan §4.1）

针孔模型 + 平面假设：镜头中心在 `(0, 0, h_cam)`，光轴沿世界 +x 方向下倾
`θ_pitch`。把"像素 (u, v, 1)" 到"地面 (x_g, y_g, 1)" 的变换写成单应::

    u' = (u - cx) / fx
    v' = (v - cy) / fy
    D  = sin θ + v' · cos θ                # 必须 > 0
    x_g =  h_cam · (cos θ - v' · sin θ) / D
    y_g = -h_cam · u' / D

令齐次分量 `w = fy · D = fy sin θ + (v − cy) cos θ`，则单应::

    H = | 0                 -h_cam sin θ              h_cam(fy cos θ + cy sin θ) |
        | -h_cam fy / fx       0                          h_cam fy cx / fx       |
        | 0                  cos θ                     fy sin θ - cy cos θ       |

`fx` / `fy` 由 `SENSOR_HFOV_DEG` / `SENSOR_VFOV_DEG` + 算法分辨率反推；
`cx, cy` 取算法分辨率正中心。yaw / 镜头畸变 / 装配垂直度都未注入——这
只是 fallback。

#### 2.3.4 RANSAC 圆弧（plan §6.3 L3a）

5 个候选点 → C(5,3)=10 个三元组，**枚举优于随机迭代**（plan §6.3 给的
"迭代 20~40 次"对随机采样有意义；N=5 时枚举更省）。对每个三元组：

1. 用代数公式（一般式 `x² + y² + D x + E y + F = 0`）求过 3 点的圆；
2. 校验半径先验 `|R − R_PRIOR| ≤ R_PRIOR_TOL_MM`（默认 ±50 mm）；不满足
   直接丢弃；
3. 统计内点数 `inliers = #{ p_i : |‖p_i − (xc, yc)‖ − R| ≤ ε }`（默认 ε=10 mm）；
4. 按 `(inliers desc, |R−R_PRIOR| asc)` 选最优。

返回的 `ArcResult.succeeded=True` 仅当 `best_inliers ≥ RANSAC_MIN_INLIERS`
（默认 3）。

#### 2.3.5 一阶 TLS 直线（plan §6.3 L3b fallback）

主方向角 `φ = 0.5 · atan2(2 Sxy, Sxx − Syy)`；切线 `(cos φ, sin φ)`，
法线 `(−sin φ, cos φ)`。强制法线朝车体 +y（左）、切线朝车头 +x（前），
让下游 `compute_path_errors_line` 的符号约定与 RANSAC 路径一致。

直道段 / RANSAC 失败时（5 点共线时圆心退化到无穷远，半径远超先验）自动
fallback。`arc_mode = "lsq"` 标记这次走的是直线路径。

#### 2.3.6 路径误差符号约定（plan §1.3 + §7.1 自洽）

plan §1.3 / §2.3 / §7.1 三处的符号描述存在字面歧义；按"控制律负反馈
自洽性"统一确定：

```text
e_y > 0  ⇔  黑线在车的右方（车体 y 轴负向）  ⇔  ω_fb = k_y · e_y > 0 → 右转纠正 ✓
ψ_e > 0  ⇔  切线方向在车头的右方                ⇔  ω_fb += k_ψ · ψ_e > 0 → 右转纠正 ✓
```

公式（推导见 `vision/geometry.py::compute_path_errors_arc` docstring）::

    d² = xc² + yc²                                              # 车体到圆心距离平方
    e_y_mm = yc · (R_prior / d − 1)                             # 偏右为正
    ψ_e_rad = atan2(sign(yc) · xc, |yc|)                        # 偏右为正

PC 端单测（`vision/geometry.py` 的镜像 / 横移 / 旋转 4 组）已校验符号
正确：圆心 (0, 400) → e_y=+9 mm（线在右）；圆心 (0, -400) → e_y=-9 mm
（线在左）；车左移 30 → e_y +30 mm；车右移 30 → e_y -30 mm；点云旋转
+0.05 rad → ψ_e=-50 mrad。

#### 2.3.7 EMA + 符号防抖（plan §7.4 + §8.1）

- `EmaEstimator(α=0.5)`：阶跃响应 4 帧到 ~94%。
- `valid=False` 时 `decay()` 而不是更新；连续 `EMA_AGE_MAX_FRAMES=5` 帧
  失效后自动 `reset()`，避免陈旧值卡死控制律。
- `SignDebounce(3)`：单帧反号被强制翻号（保留 magnitude，方向回正）；
  连续 3 帧反号才接受新基线。`x=0` 视作"无方向信息"直接放行（不增不减
  pending）。
- `PathErrorEstimator` 把 `e_y` / `ψ_e` 两路 EMA + 各自 SignDebounce 打包，
  主入口只持有一个实例。

#### 2.3.8 Q_full（plan §6.6 全权重）

```text
Q_full = w_mass    · sat(mass_total / Q_L2_MASS_NOMINAL_TOTAL, 0, 1)        · 100   (w=0.3)
       + w_geom    · sat(arc.inlier_count / arc.sample_count, 0, 1)          · 100   (w=0.3)
       + w_cont    · sat(1 − jitter_cx / Q_L2_JITTER_REF_PX, 0, 1)           · 100   (w=0.2)
       + w_r_prior · sat(1 − |R̂ − R_PRIOR| / Q_R_PRIOR_NORM_MM, 0, 1)        · 100   (w=0.2)
```

`arc=None` 或 `arc.succeeded=False` 时 `q_geom` / `q_r_prior` 都置 0，
Q_full 上限 = `0.3·100 + 0.2·100 = 50`，落到 hold/lost 分级。

`compute_q_l2` 保留不动；调用方（主入口在 `calib_mode=="none"` 时）按
需切换。

#### 2.3.9 OSD 几何可视化（`PathOverlayInfo` + `_draw_path_geometry`）

主入口每帧填一份 `PathOverlayInfo` 喂给 `Camera.render_overlay(path=...)`。
debug_overlay 内部画三层：

1. **5 点 cx 折线**（绿，宽 2 px）：连接 `detection.bands[i].cx_px @
   mid_y` 的有效点。**与 IPM 无关**，纯像素域；calib 缺失时也画，方便
   即时验证 L2 主干。
2. **圆心反投**（品红，半径 6 px）：`mapper.ground_to_pixel(arc.xc, arc.yc)`
   反投回算法坐标 → display 坐标。圆心通常在 ROI 上方很远（图像 y < 0），
   屏幕外是常态，此时画从 ROI 中心朝圆心方向的箭头（长度 ≥ 20 px）。
3. **近带切线箭头**（橙，长 200 mm 在地面坐标系下走一段后反投）：从近带
   `(cx_near, mid_y)` 出发，方向 = 当前 arc / line 切线在像素域的投影。
   `arc_mode == "ransac"` 时切线 = ⊥(P_near − C)；`arc_mode == "lsq"`
   时切线 = `LineResult.(tx, ty)`。OSD 显示优先用像素域的"近带→更远
   有效带"方向绘制箭头，缺少参考点时才回退到地面切线反投；这样在
   `CALIB:DEFAULT` 或 H 未准确标定时，箭头仍贴合屏幕上的绿色中心线。

`calib_mode == "none"` 时跳过几何（mapper=None），由文本行的红色
`NO CALIB` 提示用户。

#### 2.3.10 OSD 文本行布局（阶段 C 起）

| OSD 行 | 模板 | 触发条件 |
|---|---|---|
| 1 | `FPS xx.x  (T xx.x ms)` | 1Hz；T > FRAME_PERIOD_ALERT_MS 标红 |
| 2 | `Q xx.x  V n/N  thr nn` | 1Hz；Q < Q_HOLD 标红（Q 取 Q_full / Q_L2 视 calib_mode） |
| 3 | `e_y ±NNN mm  ψ ±NNN mr  R̂ NNN mm  in n/m  arc:ransac/lsq` | calib_mode != none |
| 3b | `NO CALIB  (run tools/calibrate_ipm.py)` | calib_mode == none，红 |
| 4a (条件) | `CALIB:DEFAULT (estimate, run calibrate_ipm.py)` | calib_mode == default，琥珀 |
| 4b (条件) | `PHOTO recal i/30` | photometric 重标定中，红 |
| 4c (条件) | `MEM used a/b KB free c KB drift x.x%` | drift ≥ 5% 或 free < 512 KB，红 |
| 4d (事件) | `BTN BIN ON/OFF` | 按键事件，1s 内可见，红 |
| 4e (条件) | `CAP n/N` | CAPTURE_ENABLE=True |

#### 2.3.11 主循环 5 s 控制台日志（plan §13.1）

```
[VLT] algo_fps=30.6 period=32.7ms frames=1234
      Q=82.3(good) V=5/5 cxN=160.4 cxF=160.7 thr=82
      e_y=+12.3mm psi_e=-45.2mrad R_hat=412 in=5/5 arc=ransac calib=calibrated
      mu=124.0 sig=18.5  mem_free=...
```

`calib_mode == "default"` 时仍打印；`calib_mode == "none"` 时
`R_hat=  -`、`in=0/0`、`arc=none`，且日志末尾追加 `HOLD(age=N)` 当 EMA
处于丢线衰减期。

### 2.4 与阶段 B 的差异

| 维度 | 阶段 B (phaseB-0.2) | 阶段 C (phaseC-0.1) |
|------|---------------------|---------------------|
| 算法层数 | L0 + L1 + L2 + Q_L2 | + IPM 5 点 + RANSAC 圆 + LSQ 直线 fallback + EMA + 符号防抖 + Q_full |
| 输出契约 | cx_px / Q_L2 / V | + e_y_mm / ψ_e_mrad / R̂_mm / inliers / arc_mode（plan §1.3 单位） |
| 标定文件 | photometric 独立 JSON | + ipm 节合并到 calib.json；schema 版本号 |
| OSD | 5 条带 + cx 圆点 + 二值 overlay | + 5 点折线 + 圆心反投 + 切线箭头 + e_y/ψ_e 文本行 |
| 模块数 | camera/photometric/line_detector/quality/debug_overlay | + ground_mapper / geometry / estimator |
| 主循环每帧 | 4 步（snapshot, photometric, detector, render） | + 4 步（IPM, RANSAC/LSQ, path errors, EMA） |
| 帧预算（plan §9.1 v2.1） | snapshot ~30ms + 算法 ≤17ms | 同；几何链路加起来 ≈ 0.4ms（5 点 IPM 0.05 + 10 三元组 RANSAC ~0.3 + EMA + Q ~0.05），不挤压预算 |

---

## 3. PC 端单测覆盖（与板端实测无关，仅算法正确性）

| 模块 | 测试场景 | 通过判据 |
|------|----------|----------|
| `config.load_calibration` | calib.json 缺失 | 返回 `{}` 不抛异常 |
| `tools/calibrate_ipm.py` | 4 对预填对应点 | DLT 解 H + 来回误差 < 1e-3 mm / px |
| `GroundMapper.load` | calib 缺失 | 进入 `default` 模式，self_test 通过 |
| `GroundMapper.load` | identity H | 进入 `calibrated`，`pixel_to_ground(10,20)=(10,20)` |
| `GroundMapper.bands_to_ground` | mock 5 条带（cx=160） | 5 个有效 GroundPoint，y_g≈0 |
| `fit_circle_3pt` | 3 点过圆 (xc=0,yc=400,R=409) | 误差 < 1e-12 |
| `fit_circle_ransac` | 5 inlier | succeeded, inliers=5/5, R=409 |
| `fit_circle_ransac` | 4 inlier + 1 离群 | succeeded, inliers=4/5 |
| `compute_path_errors_arc` | 4 组对称场景 | 符号方向（左右镜像、横移、旋转）全部符合 plan §1.3 + §7.1 自洽性 |
| `fit_line_lsq` | 5 共线点 | succeeded, residual < 1e-6 |
| `EmaEstimator` | 阶跃响应 + age 衰减 | α=0.5 4 步 = 18.75%（正确）；5 帧 invalid 后 reset |
| `SignDebounce` | 单帧 / 连续反号 | 单帧吸收，连续 3 帧接受新基线 |
| `compute_q_full` | 4 组 (perfect / arc=None / R 偏 / V=1) | 数值与公式一致 |
| 整合 pipeline | mock 5 条带 + 默认 H | 全链路无异常，e_y/ψ_e/Q_full 数值合理 |

---

## 4. 验收记录占位（实测后回填）

> 阶段 C 验收（plan §12，v2.1 errata 把端到端延迟改 ≤ 60 ms）：
> - 推车沿黑线慢速移动 → `e_y_mm` 符号稳定，不跳变；
> - 横拉车体 ±30 mm → `e_y_mm` 读数误差 ≤ ±5 mm；
> - 旋转 ±10° → `ψ_e_mrad` 响应误差 ≤ 0.03 rad（≈ 30 mrad）；
> - 端到端延迟（光斑移动 → UART 发出）≤ 60 ms（v2.1 errata）。

| 测试场景 | 配置 | 实测 e_y_mm | 实测 ψ_e_mrad | 实测 R̂_mm | 通过 |
|---------|------|--------------|---------------|------------|------|
| 推车 1/4 圈，cx_near 平滑 | profile=track, calib | _____ 跳变次数 | _____ | _____ | ☐ |
| 静态居中（应 e_y≈0 ψ_e≈0） | 同上 | _____ | _____ | _____ | ☐ |
| 横拉 +30 mm | 同上 | _____ (误差) | — | — | ☐ |
| 横拉 -30 mm | 同上 | _____ (误差) | — | — | ☐ |
| 旋转 +10° | 同上 | — | _____ (误差) | _____ | ☐ |
| 旋转 -10° | 同上 | — | _____ (误差) | _____ | ☐ |
| 端到端延迟（高速相机） | 同上 | — | — | _____ ms | ☐ |

测试方法：

1. 装车前**必须**先跑 `python tools/calibrate_ipm.py`（PC 端，4 对实测
   像素↔地面 mm）→ 拷生成的 `calib.json` 到 K230 SD 卡根；
2. 运行 `vision_line_tracking.py`，等 30 s 进入稳态；
3. 推车前进 1/4 圈，观察 OSD `e_y` / `ψ_e` 数值序列；连续 3 帧符号翻转
   计 1 次跳变（plan §8.1 的 SignDebounce 应让"非真转向"的跳变次数为 0）；
4. 用尺子横拉车体 ±30 mm（标尺画在地面，记录 OSD 读数）；
5. 用旋转台 / 转动手柄旋转车体 ±10°；
6. 端到端延迟用外部高速相机（例如手机慢动作 240 FPS）拍 LCD 屏幕的
   `e_y` 数字与人为推动的瞬间，按帧计延迟。

---

## 5. 已知问题与遗留 TODO

- [ ] **K230 端屏幕辅助打靶工具**：plan §5.2 给出了"img.rotation_corr +
  四点交互"思路。当前阶段 C 仅实现 PC 端 DLT 工作流；K230 端无鼠标，
  需要补一个屏幕十字游标 + 多按键（IO_42 已用，需再分配 2~3 个）+ 实时
  取景的标定脚本。优先级：装车后第一次标定时如果 PC 工作流够用就保持
  PC-only；如比赛现场要快速重标定再补。
- [ ] **mass_total / W_*  阈值的装车回填**（沿用阶段 B §4 TODO）：
  IPM 后近带 mm/px ≈ 1，期望黑线 18 mm 投影 ≈ 18 px；与 phaseB-0.2 的
  `W_MIN/MAX = (12, 30)`（NEAR 带）相符，但需装车实测确认。
- [ ] **ψ_e 在 yc≈0 时的退化处理**：当圆心几乎在车正前方（直道 + 圆环对
  齐）`yc < 1e-6` 时，`compute_path_errors_arc` 退化为 ψ_e=0。理论上
  这个"圆心在前方正中"工况只在车恰好位于圆环对称轴上时出现，实际很
  少见；但要在装车实测时确认是否有抖动会触发。
- [ ] **占位 H 的精度**：`MOUNT_H_CAM_MM=120` / `MOUNT_PITCH_DEG=20.0` /
  `SENSOR_HFOV_DEG=54.0` / `SENSOR_VFOV_DEG=41.0` 都是 plan §4.1 的
  推荐中位值，实际镜头 / 安装角度差 ±10° 都会让 fallback H 的 e_y 数值
  偏 几十 mm。这是 OSD 显式标 `CALIB:DEFAULT` 的根本原因；装车前不要
  用 fallback H 做控制律调参。
- [ ] **K230 ulab quirk 防御**：阶段 B task_log §4 列了多个 ulab
  `np.sum(axis=...)` / 加权质心 1-D 广播 quirk。本阶段全部走纯 Python
  标量算术（`math` + 加减乘除），不引入 ulab，**不会重蹈覆辙**。但若
  阶段 E 的 Kalman / 控制律要用 ulab 矩阵乘，需提前在主循环外做单点
  自检。
- [ ] **calib.json schema 版本兼容**：当前 `_validate_ipm_node` 仅校验
  `H_3x3` 长度 9 + image_wh 匹配。后续阶段 D 引入 UART 协议版本号后，
  整个 calib.json 的 schema 版本要与 CONFIG_VERSION 联调。
- [ ] **Q 分级阈值在 calib_mode 切换时的兼容**：calib=none 时主入口用
  Q_L2（不含 geom/r_prior），可能 80+；calib=calibrated 但 arc 失败时
  用 Q_full（geom/r_prior=0），上限 50。两边的 grade() 含义不一致——
  阶段 D 接 UART 时需要决定把哪个 Q 作为协议字段（plan §10.5 `quality`）。
  目前主入口控制台日志 / OSD 都按 calib_mode 切换，是临时方案。
- [ ] **debug_overlay 切线箭头在直道 / lsq 模式下的方向退化**：当所有
  扫描带 cx 几乎相等时（直道近车段），LSQ 直线主方向角 `φ` 不稳定（`Sxx`
  与 `Syy` 都很小）；当前 `fit_line_lsq` 在 `Sxx + Syy < 1e-9` 时返回
  `succeeded=False`，OSD 文本行回退 `arc:none`、不画切线箭头。装车实测
  时确认这个阈值是否需要调。

---

## 6. 进入下一阶段的前置条件

**代码侧已可进入阶段 D**（UART 链路与仿真循环）。下列实测项不阻塞代码
推进，但**完成赛道实物前**必须补齐：

1. §4 验收表 7 行全部回填实测数据；
2. 装车后用 `tools/calibrate_ipm.py` 生成真实 `calib.json`，commit 一份
   占位（不进 git，但模板留在 docs/）；
3. 至少 3 段视频（推车 1/4 圈 / 横拉 / 旋转），存档到 `tests/golden/`，
   作为阶段 D / E 的回归对照；
4. 把实测后的 `MOUNT_H_CAM_MM` / `MOUNT_PITCH_DEG` 真值写回
   `docs/hardware_profile.md` 与 `config.py` 注释。

---

## 7. 给阶段 D 的接力条目

- **`MOTION_CMD` 字段映射**（plan §10.5）：
  - `e_y` (int16, mm) ← `path_overlay.e_y_filt_mm`，clip 到 ±300，乘 1
  - `heading_error` (int16, mrad) ← `path_overlay.psi_e_filt_mrad`，
    clip 到 ±1570
  - `quality` (uint8) ← `q_full * 100 / 100`（已 0~100）；calib_mode 切换
    时见 §5 的 Q 兼容 TODO。
  - `mode` (uint8)：阶段 E 控制律落地时再决定（current "track" 或 "stop"）；
    阶段 D 暂置 `1`（track）但实际不消费。
  - `flags` (位域)：
    - `flags.degrade` ← `q_full < 80` (Q_DEGRADE)
    - `flags.lost` ← `not path_overlay.valid` 或 `e_y_age >= EMA_AGE_MAX_FRAMES`
    - `flags.calib_change` ← 阶段 C 当前未实现 photometric/IPM 重标定通知，
      留给阶段 D 在 UART 帧打包前补。
- **`v_ref` / `omega_ref` 不在阶段 C 计算**：阶段 D 仅做 UART 链路 + 心跳
  + 失联降级；`v_ref` / `omega_ref` 留 0（或主控自定义默认）；阶段 E 接
  controller 后才填实际值。
- **`PathOverlayInfo` 中的字段**与 UART `MOTION_CMD` PAYLOAD 一一对应，
  阶段 D 的 frame.py / protocol.py 可直接读 path_overlay。
- **`calib.json` 缺失时的协议字段**：建议把 `flags.degrade=1` 立即拉起，
  并把 `quality=0`，让主控在第一时间感知"K230 没有有效标定"。
- **K230 → 主控心跳**：阶段 D 起需要在主入口主循环之外起一个低频任务
  （plan §10.7 10 Hz），与现有 `LOG_INTERVAL_MS=5000` 解耦；也可以借用
  `maybe_update_fps`（1 Hz）的回调，但 plan 心跳 10 Hz 更合理。

---

## 8. 日志与 OSD 字段含义速查（阶段 C 增量）

### 8.1 主循环 5 s 日志新增字段

```
[VLT] ... e_y=+12.3mm psi_e=-45.2mrad R_hat=412 in=5/5 arc=ransac calib=calibrated HOLD(age=2) ...
```

| 字段 | 单位 | 含义 | 健康范围 |
|---|---|---|---|
| `e_y` | mm | EMA 滤波 + 符号防抖后的横向偏差（正号 = 黑线在车右） | ±300 mm（plan §1.3） |
| `psi_e` | mrad | EMA 滤波后的航向误差（正号 = 切线在车头右） | ±1570 mrad |
| `R_hat` | mm | RANSAC 圆弧拟合半径；`-` 表示直线 / 失败 | 359~459 mm（先验 ±50） |
| `in` | n/m | RANSAC 内点 / 总样本数 | 4/5 ~ 5/5 |
| `arc` | str | `ransac` / `lsq` / `none`，本帧拟合模式 | ransac 优于 lsq |
| `calib` | str | `calibrated` / `default` / `none` | calibrated 为目标 |
| `HOLD(age=N)` | 帧 | EMA 处于丢线衰减期的帧数 | 不出现为佳；> 5 触发 reset |

### 8.2 OSD 文本行（已在 §2.3.10 列出）

### 8.3 OSD 几何叠加颜色

| 元素 | 颜色 | RGB | config 字段 |
|---|---|---|---|
| 5 点 cx 折线 | 绿 | (0, 255, 0) | `OSD_PATH_COLOR` |
| 圆心反投点 / 边缘箭头 | 品红 | (255, 0, 255) | `OSD_CIRCLE_CENTER_COLOR` |
| 近带切线箭头 | 橙 | (255, 128, 0) | `OSD_TANGENT_COLOR` |
| `CALIB:DEFAULT` 文字 | 琥珀 | (255, 200, 0) | `OSD_CALIB_DEFAULT_COLOR` |
| `NO CALIB` 文字 | 红 | (255, 0, 0) | `OSD_NO_CALIB_COLOR` |

### 8.4 启动期一次性日志（阶段 C 增量）

```
[VLT] vision_line_tracking start, config=phaseC-0.1, debug=True, profile=bench
[VLT] L2 thresholds: ...
[VLT] L2 segment select: ...
[VLT] IPM mode: default                                          ← 阶段 C 新增
[VLT] CALIB:DEFAULT — e_y/ψ_e values carry tens-of-mm system bias.    ← 阶段 C 新增
[VLT]   to fix: run tools/calibrate_ipm.py on PC, copy calib.json to /sdcard/
[camera] binary overlay setup: ...
[camera] request: sensor=1280x720@60, ...
[photometric] bootstrap done frames=30 ...
```
