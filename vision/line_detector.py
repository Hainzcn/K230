"""黑线检测主流水线（plan §6.1 L0 + L1 + L2，phaseB-0.2 段选择版）。

数据流（一帧）::

    snapshot CHN1 (320x240 GRAYSCALE)
      └─► cv_lite.grayscale_threshold_binary(thresh, 255)        # L0
            ├─► bin_raw  shape=(H, W) uint8，黑线=0 背景=255
            └─► bin_inv = 255 − bin_raw_roi                       # 反相，前景=255
                  └─► (可选) image.Image(ALLOC_REF) → open(1)    # L1
                        └─► 5 条扫描带 ulab 切片 + 列向量求和    # L2.a
                              └─► col_sum (W,) → bytes 一次解码 # L2.b
                                    └─► find_runs(>thr,gap_tol)  # L2.c
                                          └─► 候选筛选 (W/MASS)  # L2.d
                                                └─► 选段排序：    # L2.e
                                                    1. 时域 prior
                                                    2. 空间 prior
                                                    3. mass 兜底
                                                    └─► BandResult
                                                          └─► Q_L2

设计要点：

- L0 的极性：``cv_lite.grayscale_threshold_binary`` 是 ``pix > thresh → 255``，
  对暗黑线会输出 0。我们要 "黑线 = 前景 = 255" 才能直接对列求和拿到密度，
  所以做一次 ``bin_inv = 255 − bin_raw_roi``（仅在 ROI 上做，省一半开销）。

- L1 双后端：
  - ``"image_open"``：``image.Image(ALLOC_REF)`` 包装后调 ``img.open(1)``
    （OpenMV 原生 erode+dilate，3×3/1 iter）。
  - ``"none"``：跳过形态学，仅靠 L2 段筛选去噪。
  通过 ``config.LINE_L1_BACKEND`` 切换；运行期不动态切换。

- **L2 段选择（phaseB-0.2 起）**：旧实现是 ``cx = Σ x·col_sum / Σ col_sum``
  全列加权质心——ROI 出现"另一块黑"（路面碎屑 / 阴影 / 桌面异色）会被
  按质量加权拉偏，控制律失稳。新实现把 col_sum 切成连续段（>阈值的列
  簇），按几何 + 时域 + 空间三道 prior 选最贴合"上一帧那条线"的段，干扰
  段在选段阶段就被排除，不污染 cx。

- L2 全部走 ``bytes(col_sum)`` 一次物化后扫描（plan §9.2 守则 3）。
  ulab 在 K230 上的 ``np.sum(axis=...)`` / 加权求和有标量广播 quirk
  （详见 task_log §4 已知问题），我们直接绕开。

- 带索引约定：i=0 是 y_top 最小（最远处），i=BAND_COUNT-1 是 y_top 最大
  （最近处）。``bands[-1]`` 永远是近带；选段传播按 NEAR→FAR（``range(N-1, -1, -1)``）
  的顺序进行，因为 NEAR 是 e_y 主源、置信最高（plan §4.3）。
"""

import image
import cv_lite
from ulab import numpy as np

import config
from vision.interrupts import reraise_if_stop
from vision.quality import compute_q_l2


def _byte_to_int(b):
    """MicroPython / CPython 兼容地读取 bytes 单项。"""
    if isinstance(b, int):
        return b
    return ord(b)


def _decode_col_sum_bytes(col_sum, roi_w):
    """把 col_sum 通过 ``bytes()`` 解码为长度 ``roi_w`` 的 Python int 列表。

    K230 ulab 上 col_sum 是 uint8 行的累加结果，dtype 升级为 int16/uint16，
    每列在 bytes 里占 ``stride = len(raw) // roi_w`` 字节，小端解码。

    返回 ``None`` 表示解码失败（罕见，仅在 ulab 行为再变时触发）。
    """
    try:
        raw = bytes(col_sum)
    except Exception:
        return None
    n = len(raw)
    if roi_w <= 0 or n < roi_w:
        return None
    stride = n // roi_w
    if stride <= 0:
        return None

    vals = [0] * roi_w
    off = 0
    for x in range(roi_w):
        v = 0
        shift = 0
        end = off + stride
        while off < end:
            v += _byte_to_int(raw[off]) << shift
            shift += 8
            off += 1
        vals[x] = v
    return vals


def _find_runs(col_vals, threshold, gap_tol):
    """在 1-D ``col_vals`` 上找所有 ``v > threshold`` 的连续段。

    允许 ``gap_tol`` 列以内的"低于阈值"被桥接到同一段，避免 sensor 噪声 /
    抗锯齿把单段断成多段。返回 list[(x_left, x_right_excl, mass, weighted)]。

    - ``mass = Σ v``，仅累加段内（含被桥接的窄洞列，让 cx 还原回真实质心）。
    - ``weighted = Σ x · v``，与 mass 配对算 ``cx = weighted / mass``。
    - ``x_right_excl - x_left = width_px``（段宽度，含桥接列）。

    复杂度 O(W)；W=320, gap_tol=1 时实测每带 < 0.2 ms。
    """
    runs = []
    if not col_vals:
        return runs

    n = len(col_vals)
    in_run = False
    run_start = 0
    run_mass = 0
    run_weighted = 0
    last_above_x = -1
    pending_mass = 0
    pending_weighted = 0
    gap_count = 0

    for x in range(n):
        v = col_vals[x]
        if v > threshold:
            if not in_run:
                in_run = True
                run_start = x
                run_mass = 0
                run_weighted = 0
                pending_mass = 0
                pending_weighted = 0
                gap_count = 0
            else:
                run_mass += pending_mass
                run_weighted += pending_weighted
                pending_mass = 0
                pending_weighted = 0
                gap_count = 0
            run_mass += v
            run_weighted += x * v
            last_above_x = x
        else:
            if in_run:
                pending_mass += v
                pending_weighted += x * v
                gap_count += 1
                if gap_count > gap_tol:
                    runs.append(
                        (run_start, last_above_x + 1, run_mass, run_weighted)
                    )
                    in_run = False
                    pending_mass = 0
                    pending_weighted = 0
                    gap_count = 0

    if in_run:
        runs.append(
            (run_start, last_above_x + 1, run_mass, run_weighted)
        )
    return runs


def _select_best_run(runs, min_mass, w_min, w_max,
                     prev_cx, neighbor_cx, prior_radius, dcx_max):
    """从候选段里按"几何 + 时域 + 空间 + mass"四道筛选选最优段。

    :param runs:       :func:`_find_runs` 返回的 (x_l, x_r, mass, weighted) 列表
    :param min_mass:   该带的 ``MIN_MASS_PER_BAND[i]``
    :param w_min/max:  该带的 ``W_MIN/MAX_PX_PER_BAND[i]``
    :param prev_cx:    上一帧本带选中的 cx（``-1`` 表示无 / 已过期）
    :param neighbor_cx: NEAR→FAR 传播链中上一带本帧选中的 cx（``-1`` 表示无）
    :param prior_radius: 时域 prior 半径（``LINE_CX_PRIOR_RADIUS_PX``）
    :param dcx_max:    空间 prior 半径（``DELTA_CX_MAX_PX``）

    :return: 选中的 ``(x_l, x_r, mass, weighted, cx, width)`` 元组；
             无候选返回 ``None``。元组里把 ``cx`` 与 ``width`` 一并算好，
             调用方不再除一次法。

    选段优先级（前一档命中即返回）：

    1. **时域 prior**：``|cx - prev_cx| ≤ prior_radius`` 的候选里取距离最小者。
       ``prev_cx < 0`` 跳过该档。
    2. **空间 prior**：``|cx - neighbor_cx| ≤ dcx_max`` 的候选里取距离最小者。
       ``neighbor_cx < 0`` 跳过该档（NEAR 第一带或前面带未选中时）。
    3. **mass 兜底**：取剩余候选中 ``mass`` 最大者。

    几何 + mass 是硬筛（在 1 之前先过一遍），不满足直接出局——这就是把
    plan §6.2 的 ``MIN_MASS / W_MIN / W_MAX`` 三道硬约束从"事后剔除"
    挪到"选段前候选过滤"，让干扰段在选段阶段就消失，不污染 cx。
    """
    if not runs:
        return None

    candidates = []
    for r in runs:
        x_l, x_r, mass, weighted = r
        w = x_r - x_l
        if w < w_min or w > w_max:
            continue
        if mass < min_mass:
            continue
        cx = weighted / float(mass)
        candidates.append((r, cx, w, mass))

    if not candidates:
        return None

    if prev_cx >= 0.0:
        best = None
        best_d = prior_radius + 1.0
        for c in candidates:
            d = c[1] - prev_cx
            if d < 0.0:
                d = -d
            if d <= prior_radius and d < best_d:
                best = c
                best_d = d
        if best is not None:
            r, cx, w, mass = best
            return (r[0], r[1], r[2], r[3], cx, w)

    if neighbor_cx >= 0.0:
        best = None
        best_d = dcx_max + 1.0
        for c in candidates:
            d = c[1] - neighbor_cx
            if d < 0.0:
                d = -d
            if d <= dcx_max and d < best_d:
                best = c
                best_d = d
        if best is not None:
            r, cx, w, mass = best
            return (r[0], r[1], r[2], r[3], cx, w)

    best = candidates[0]
    for c in candidates[1:]:
        if c[3] > best[3]:
            best = c
    r, cx, w, mass = best
    return (r[0], r[1], r[2], r[3], cx, w)


class BandResult:
    """单条扫描带的检测结果。

    所有像素坐标都在算法分辨率（``ALGO_WIDTH × ALGO_HEIGHT``）下。

    phaseB-0.2 起新增 ``cx_prev`` / ``cx_prev_age`` 用于时域 prior：
    - ``cx_prev``：上一帧本带选中的 cx（无效时 ``-1.0``）
    - ``cx_prev_age``：自 ``cx_prev`` 设值以来连续无效的帧数；超过
      ``LINE_CX_PRIOR_AGE_MAX_FRAMES`` 即视为过期，不再参与本帧选段。
      这两个字段由 :class:`LineDetector` 跨帧维护，OSD / quality 不消费。
    """

    __slots__ = (
        "idx",
        "y_top",
        "y_bot",
        "mass",
        "cx_px",
        "width_px",
        "valid",
        "reject",
        "cx_prev",
        "cx_prev_age",
    )

    def __init__(self, idx, y_top, y_bot):
        self.idx = idx
        self.y_top = y_top
        self.y_bot = y_bot
        self.mass = 0.0
        self.cx_px = -1.0
        self.width_px = 0
        self.valid = False
        self.reject = ""
        self.cx_prev = -1.0
        self.cx_prev_age = 0

    def __repr__(self):
        return (
            "BandResult(idx=%d, y=[%d,%d], mass=%.0f, cx=%.1f, w=%d, "
            "valid=%s, reject=%s)"
            % (
                self.idx,
                self.y_top,
                self.y_bot,
                self.mass,
                self.cx_px,
                self.width_px,
                self.valid,
                self.reject or "-",
            )
        )


class DetectionResult:
    """一帧的完整检测输出。

    属性：

    - ``bands`` (list[BandResult]) ：5 条带的结果，索引 0=远 -> 4=近
    - ``binary_np`` (ndarray)      ：``bin_inv_roi``，前景=255、背景=0；
      L2 / OSD / 阶段 C ground_mapper 都从这里取数据。
    - ``binary_image`` (image.Image | None)：历史接口，OSD 旧路径需要 MMZ
      backed image 做 ``draw_image`` source；新路径 OSD 走 ``bytes(binary_np)``
      字节扫描，已删除 MMZ 镜像，本字段恒为 None，仅保留以兼容外部读者。
    - ``q_l2`` (float)             ：仅用 L2 子项的 Q 评分（0~100）
    - ``mass_total`` (float)       ：5 条带选中段 mass 之和
    - ``n_valid`` (int)            ：通过段筛选 + prior 选中的带数
    - ``cx_near_px`` (float)       ：最近带的 cx；若无效返回 -1
    - ``cx_far_px`` (float)        ：最远带的 cx；若无效返回 -1
    - ``threshold_used`` (int)     ：本次 L0 用的阈值（光度漂移诊断）
    """

    __slots__ = (
        "bands",
        "binary_np",
        "binary_image",
        "q_l2",
        "mass_total",
        "n_valid",
        "cx_near_px",
        "cx_far_px",
        "threshold_used",
    )

    def __init__(self):
        self.bands = []
        self.binary_np = None
        self.binary_image = None
        self.q_l2 = 0.0
        self.mass_total = 0.0
        self.n_valid = 0
        self.cx_near_px = -1.0
        self.cx_far_px = -1.0
        self.threshold_used = 0


class LineDetector:
    """L0 + L1 + L2 流水线（phaseB-0.2 段选择版）。

    构造期：从 ``config`` 读取所有参数，预创建 ROI 切片元数据；主循环
    ``process(img, threshold)`` 不再创建可复用对象（plan §9.2 守则 5）。

    跨帧状态：每条 ``BandResult`` 持有 ``cx_prev`` / ``cx_prev_age``，
    供下一帧的时域 prior 使用；其余状态全部一次性。
    """

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else config
        self.W = self.cfg.ALGO_WIDTH
        self.H = self.cfg.ALGO_HEIGHT
        self._image_shape = [self.H, self.W]

        roi_x, roi_y, roi_w, roi_h = self.cfg.ROI_TOTAL_PX
        if roi_x != 0 or roi_w != self.W:
            print(
                "[line_detector] WARN: ROI_TOTAL_PX x/w mismatch (%d, %d) "
                "vs algo width %d; cx 计算仍按全 W 起点 0 进行"
                % (roi_x, roi_w, self.W)
            )
        self._roi_x = roi_x
        self._roi_y = roi_y
        self._roi_w = roi_w
        self._roi_h = roi_h

        self._band_count = self.cfg.BAND_COUNT
        self._band_h = self.cfg.BAND_HEIGHT_PX
        self._band_tops = list(self.cfg.BAND_TOPS_PX)
        if len(self._band_tops) != self._band_count:
            raise ValueError(
                "BAND_TOPS_PX length %d != BAND_COUNT %d"
                % (len(self._band_tops), self._band_count)
            )
        self._band_tops_rel = [t - self._roi_y for t in self._band_tops]
        for i, t in enumerate(self._band_tops_rel):
            if t < 0 or t + self._band_h > self._roi_h:
                raise ValueError(
                    "band %d top=%d out of ROI vertical range [0,%d]"
                    % (i, t, self._roi_h - self._band_h)
                )

        self._min_mass = list(self.cfg.MIN_MASS_PER_BAND)
        self._w_min = list(self.cfg.W_MIN_PX_PER_BAND)
        self._w_max = list(self.cfg.W_MAX_PX_PER_BAND)
        for arr, name in (
            (self._min_mass, "MIN_MASS_PER_BAND"),
            (self._w_min, "W_MIN_PX_PER_BAND"),
            (self._w_max, "W_MAX_PX_PER_BAND"),
        ):
            if len(arr) != self._band_count:
                raise ValueError(
                    "%s length %d != BAND_COUNT %d"
                    % (name, len(arr), self._band_count)
                )

        self._delta_cx_max = float(self.cfg.DELTA_CX_MAX_PX)
        self._col_sum_thr = self.cfg.COL_SUM_THR_FOR_WIDTH
        self._l1_backend = self.cfg.LINE_L1_BACKEND

        self._gap_tol = int(self.cfg.get("LINE_RUN_GAP_TOLERANCE_PX", 0))
        if self._gap_tol < 0:
            self._gap_tol = 0
        self._prior_radius = float(
            self.cfg.get("LINE_CX_PRIOR_RADIUS_PX", self._delta_cx_max)
        )
        self._prior_age_max = int(
            self.cfg.get("LINE_CX_PRIOR_AGE_MAX_FRAMES", 5)
        )
        if self._prior_age_max < 0:
            self._prior_age_max = 0

        # 历史：早期 OSD 通过 ``image.draw_image`` 画 binary，必须有一个 MMZ
        # 分配的 GRAYSCALE image.Image 作为 source。当时这里持有
        # ``self._roi_img = image.Image(roi_w, roi_h, image.GRAYSCALE)``，
        # process() 末尾再 ``_roi_img.copy_from(src_wrap)`` 把 bin_inv_roi
        # 搬进 MMZ，每帧 ~48 KB memcpy ≈ 1-2 ms。
        #
        # 现在 OSD 改走 ``bytes(detection.binary_np)`` + Python 字节扫描，
        # 不再消费 binary_image，因此 _roi_img + copy_from 整条路径删掉，
        # 主路径每帧省 1-2 ms。binary_image 字段保留但置 None，向后兼容。

        self._result = DetectionResult()
        self._result.bands = [
            BandResult(
                idx=i,
                y_top=self._band_tops[i],
                y_bot=self._band_tops[i] + self._band_h,
            )
            for i in range(self._band_count)
        ]
        self._result.binary_image = None

        # process 中临时复用：每带的 col_vals（Python int list）与 runs（tuple list）
        # 不能跨帧持有（下一帧立刻被覆盖），但每帧分配新 list 比每带分配
        # 一次再清空更便宜（短 list 在 MicroPython 上 GC 友好）。
        self._scratch_runs = [None] * self._band_count

    # ------------------------------------------------------------------ #
    # 主入口
    # ------------------------------------------------------------------ #
    def process(self, img, threshold):
        """对一帧 GRAYSCALE 图像执行 L0+L1+L2 流水线。

        :param img: ``image.Image`` (CHN1 grayscale snapshot)
        :param threshold: 当前 line_threshold（来自 :class:`Photometric`）
        :return: 内部复用的 :class:`DetectionResult`
        """
        result = self._result
        result.threshold_used = threshold
        result.q_l2 = 0.0
        result.mass_total = 0.0
        result.n_valid = 0
        result.cx_near_px = -1.0
        result.cx_far_px = -1.0

        if img is None:
            self._reset_bands_invalid("no_image")
            self._decay_priors_all()
            return result

        img_np = img.to_numpy_ref()

        # ---------- L0: cv_lite 二值化 ---------- #
        try:
            bin_raw = cv_lite.grayscale_threshold_binary(
                self._image_shape, img_np, int(threshold), 255
            )
        except Exception as e:
            reraise_if_stop(e)
            print("[line_detector] L0 binarize failed:", e)
            self._reset_bands_invalid("L0_fail")
            self._decay_priors_all()
            return result

        # ---------- 反相到 ROI（黑线 = 前景 = 255） ---------- #
        # 仅对 ROI 行做反相，省一半 alloc / 算力。bin_inv_roi 是 heap-allocated
        # 的 ulab ndarray；L2 直接消费它，OSD 通过 ``bytes(bin_inv_roi)``
        # 物化后扫描。
        bin_inv_roi = 255 - bin_raw[self._roi_y : self._roi_y + self._roi_h, :]
        result.binary_np = bin_inv_roi

        # ---------- L1: 形态学开运算（可选） ---------- #
        if self._l1_backend == "image_open":
            try:
                src_wrap = image.Image(
                    self._roi_w, self._roi_h, image.GRAYSCALE,
                    alloc=image.ALLOC_REF, data=bin_inv_roi,
                )
                src_wrap.open(1)
            except Exception as e:
                reraise_if_stop(e)
                print("[line_detector] L1 open(1) failed:", e)

        # ---------- L2.a/b/c: 列向求和 + bytes 解码 + 段查找 ---------- #
        # 先把 5 条带的 col_sum 各算一次、解码成 Python int list、找出 runs。
        # 选段（依赖跨带 prior）放到第二轮做。
        bands = result.bands
        for i in range(self._band_count):
            band = bands[i]
            band.valid = False
            band.reject = ""
            band.mass = 0.0
            band.cx_px = -1.0
            band.width_px = 0

            y_top_rel = self._band_tops_rel[i]
            band_slice = bin_inv_roi[y_top_rel : y_top_rel + self._band_h, :]

            # 列向求和：8 行 × 320 列 → 320 长向量。
            #
            # **K230 ulab quirk**：``np.sum(band_slice, axis=0)`` 在 K230
            # 当前固件上忽略 axis 参数，对 (8, 320) 输入返回标量总和。
            # 修复：手工按行累加 ``band_slice[r]``。BAND_HEIGHT_PX=8 总共
            # 7 次加法 ≈ 0.3ms。详见 task_log §4 已知问题。
            col_sum = band_slice[0] + band_slice[1]
            for r in range(2, self._band_h):
                col_sum = col_sum + band_slice[r]

            try:
                if len(col_sum) != self._roi_w:
                    band.reject = "col_sum_shape"
                    self._scratch_runs[i] = None
                    continue
            except TypeError:
                band.reject = "col_sum_scalar"
                self._scratch_runs[i] = None
                continue

            col_vals = _decode_col_sum_bytes(col_sum, self._roi_w)
            if col_vals is None:
                band.reject = "decode_fail"
                self._scratch_runs[i] = None
                continue

            self._scratch_runs[i] = _find_runs(
                col_vals, self._col_sum_thr, self._gap_tol
            )

        # ---------- L2.d/e: 选段（NEAR→FAR 传播 + 时空 prior） ---------- #
        # 顺序：bands[-1] (NEAR) → bands[0] (FAR)。NEAR 是 e_y 主源、置信
        # 最高，先选定后给后续带提供空间 prior。
        valid_count = 0
        mass_total = 0.0
        prev_neighbor_cx = -1.0  # 传播链中"上一带本帧选中的 cx"

        for i in range(self._band_count - 1, -1, -1):
            band = bands[i]
            runs = self._scratch_runs[i]
            if runs is None:
                # 已在前一阶段标记 reject，prior 衰减
                band.cx_prev_age += 1
                if band.cx_prev_age > self._prior_age_max:
                    band.cx_prev = -1.0
                continue

            if not runs:
                band.reject = "no_runs"
                band.cx_prev_age += 1
                if band.cx_prev_age > self._prior_age_max:
                    band.cx_prev = -1.0
                continue

            prev_cx_for_select = -1.0
            if (band.cx_prev >= 0.0
                    and band.cx_prev_age <= self._prior_age_max):
                prev_cx_for_select = band.cx_prev

            picked = _select_best_run(
                runs,
                self._min_mass[i],
                self._w_min[i],
                self._w_max[i],
                prev_cx_for_select,
                prev_neighbor_cx,
                self._prior_radius,
                self._delta_cx_max,
            )

            if picked is None:
                # 最常见三种原因：mass<MIN、width 不在 [W_MIN, W_MAX]、
                # 时空 prior 都把它挡了。把最具诊断价值的"最大 mass 候选
                # 但 width / mass 不达标"压成一行 reject。
                band.reject = self._diagnose_no_pick(
                    runs, i
                )
                band.cx_prev_age += 1
                if band.cx_prev_age > self._prior_age_max:
                    band.cx_prev = -1.0
                continue

            x_l, x_r, mass_seg, weighted_seg, cx, width = picked
            band.mass = float(mass_seg)
            band.cx_px = float(cx)
            band.width_px = int(width)
            band.valid = True
            band.cx_prev = float(cx)
            band.cx_prev_age = 0

            valid_count += 1
            mass_total += float(mass_seg)
            prev_neighbor_cx = float(cx)

        result.mass_total = mass_total
        result.n_valid = valid_count

        if bands[-1].valid:
            result.cx_near_px = bands[-1].cx_px
        if bands[0].valid:
            result.cx_far_px = bands[0].cx_px

        result.q_l2 = compute_q_l2(result, self.cfg)
        return result

    # ------------------------------------------------------------------ #
    # 辅助
    # ------------------------------------------------------------------ #

    def _diagnose_no_pick(self, runs, band_idx):
        """选段失败时挑出"信息最多"的 reject 字段。

        逻辑：在所有 run 里挑 mass 最大的那个，按它失败的具体维度命名。
        让日志直接告诉你"是 mass 不够、width 太宽、还是被 prior 挡了"。
        """
        if not runs:
            return "no_runs"
        best = runs[0]
        for r in runs[1:]:
            if r[2] > best[2]:
                best = r
        x_l, x_r, mass, weighted = best
        w = x_r - x_l
        min_mass = self._min_mass[band_idx]
        w_min = self._w_min[band_idx]
        w_max = self._w_max[band_idx]
        if mass < min_mass:
            return "mass<%d" % min_mass
        if w < w_min:
            return "w<%d" % w_min
        if w > w_max:
            return "w>%d" % w_max
        return "prior_reject"

    def _decay_priors_all(self):
        """所有带的 cx_prev_age++；超过阈值则丢弃 prior。

        异常路径（snapshot 失败 / L0 失败）调用，让 prior 不会"卡在"
        几帧前的位置。
        """
        for band in self._result.bands:
            band.cx_prev_age += 1
            if band.cx_prev_age > self._prior_age_max:
                band.cx_prev = -1.0

    def _reset_bands_invalid(self, reason):
        for band in self._result.bands:
            band.valid = False
            band.reject = reason
            band.mass = 0.0
            band.cx_px = -1.0
            band.width_px = 0

    def self_test(self):
        """校验配置一致性。运行期 process() 之前调用。"""
        if self._band_count != len(self._result.bands):
            return False
        if self._l1_backend not in ("image_open", "none"):
            print("[line_detector.self_test] unknown L1 backend:", self._l1_backend)
            return False
        for i, t in enumerate(self._band_tops_rel):
            if t < 0 or t + self._band_h > self._roi_h:
                print(
                    "[line_detector.self_test] band %d out of ROI" % i
                )
                return False
        if self._gap_tol < 0:
            print("[line_detector.self_test] gap_tol must be >= 0")
            return False
        return True
