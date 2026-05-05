"""黑线检测主流水线（plan §6.1 L0 + L1 + L2）。

数据流（一帧）::

    snapshot CHN1 (320x240 GRAYSCALE)
      └─► cv_lite.grayscale_threshold_binary(thresh, 255)        # L0
            ├─► bin_raw  shape=(H, W) uint8，黑线=0 背景=255
            └─► bin_inv = 255 − bin_raw_roi                       # 反相，前景=255
                  └─► (可选) image.Image(ALLOC_REF) → open(1)    # L1
                        └─► 5 条扫描带 ulab 切片 + 列向量求和    # L2
                              ├─► col_sum / mass / cx / width
                              └─► 硬约束过滤 → BandResult
                                    └─► Q_L2 由 vision.quality 计算

设计要点：

- L0 的极性：``cv_lite.grayscale_threshold_binary`` 是 ``pix > thresh → 255``，
  对暗黑线会输出 0。我们要 "黑线 = 前景 = 255" 才能直接对列求和拿到密度，
  所以做一次 ``bin_inv = 255 − bin_raw_roi``（仅在 ROI 上做，省一半开销）。

- L1 双后端（plan + 用户选择）：
  - ``"image_open"``：把 ``bin_inv_roi`` 通过 ``image.Image(ALLOC_REF)`` 零拷贝
    包装后调 ``img.open(1)``（OpenMV 原生 erode+dilate，3×3/1 iter）。
  - ``"none"``：跳过形态学，仅靠 L2 硬约束去噪。
  通过 ``config.LINE_L1_BACKEND`` 切换；运行期不动态切换。

- L2 全部走 ``ulab.numpy`` 向量化，禁止逐像素 Python 循环（plan §9.2 守则 3）。
  ``arange_x`` 在 ``__init__`` 预创建为浮点 ndarray，每帧只做切片 + sum + 标量运算。

- 带索引约定：i=0 是 y_top 最小（最远处），i=BAND_COUNT-1 是 y_top 最大（最近处）。
  这样 ``bands[-1]`` 永远是近带，在 e_y 里权重最高（plan §4.3）。
"""

import image
import cv_lite
from ulab import numpy as np

import config
from vision.interrupts import reraise_if_stop
from vision.quality import compute_q_l2


def _scalar_to_int(v):
    """把 K230 ulab 迭代出的标量转成 Python int。

    部分固件上 ``for v in ndarray`` 会给出普通数字；也有固件会给出
    ``'\x00\x00'`` 这类 2 字节小端字符串。这里统一解成整数。
    """
    try:
        return int(v)
    except Exception:
        pass
    if isinstance(v, str):
        n = 0
        shift = 0
        for ch in v:
            n += ord(ch) << shift
            shift += 8
        return n
    try:
        n = 0
        shift = 0
        for b in v:
            n += int(b) << shift
            shift += 8
        return n
    except Exception:
        return 0


def _byte_to_int(b):
    """MicroPython / CPython 兼容地读取 bytes 单项。"""
    if isinstance(b, int):
        return b
    return ord(b)


def _col_sum_stats_from_bytes(col_sum, roi_w, threshold):
    """用原始 bytes 快速计算 col_sum 的 width 与加权和。

    K230 上直接迭代 ulab 标量会产出 ``'\x00\x00'``，逐点转换很慢；bytes()
    后按小端整数解码，能避免每列一次异常。
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

    width = 0
    weighted = 0.0
    off = 0
    for x in range(roi_w):
        v = 0
        shift = 0
        end = off + stride
        while off < end:
            v += _byte_to_int(raw[off]) << shift
            shift += 8
            off += 1
        if v > threshold:
            width += 1
        weighted += x * v
    return width, weighted


class BandResult:
    """单条扫描带的检测结果。

    所有像素坐标都在算法分辨率（``ALGO_WIDTH × ALGO_HEIGHT``）下。
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
    - ``mass_total`` (float)       ：5 条带 mass 之和
    - ``n_valid`` (int)            ：通过硬约束的带数
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
    """L0 + L1 + L2 流水线。

    构造期：从 ``config`` 读取所有参数，预创建 arange ndarray 与 ROI 切片元数据，
    主循环 ``process(img, threshold)`` 不再创建可复用对象（plan §9.2 守则 5）。
    """

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else config
        self.W = self.cfg.ALGO_WIDTH
        self.H = self.cfg.ALGO_HEIGHT
        # cv_lite image_shape 顺序为 [H, W]
        self._image_shape = [self.H, self.W]

        roi_x, roi_y, roi_w, roi_h = self.cfg.ROI_TOTAL_PX
        # 阶段 B 假设 ROI x 起点为 0、宽度等于 ALGO_WIDTH（plan §4.3 列等高网格）。
        # 万一以后改成左右梯形掩膜，此处再扩。
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
        # ROI 内的相对 y_top（用于切片 bin_inv_roi）
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

        self._delta_cx_max = self.cfg.DELTA_CX_MAX_PX
        self._col_sum_thr = self.cfg.COL_SUM_THR_FOR_WIDTH
        self._l1_backend = self.cfg.LINE_L1_BACKEND

        # 预创建 arange，避免帧循环 alloc。强制为浮点防止后续乘法 dtype 升级抖动。
        self._arange_x = np.arange(self.W) * 1.0
        # K230 实板一旦发现 ulab 的 bool sum / 加权 sum 语义异常，就切到
        # bytes(col_sum) 解析；避免之后每条带都先走一次已知会失败的 ulab 路径。
        self._prefer_bytes_col_stats = False

        # 历史：早期 OSD 通过 ``image.draw_image`` 画 binary，必须有一个 MMZ
        # 分配的 GRAYSCALE image.Image 作为 source（因为 ALLOC_REF wrap 在
        # K230 上作为 draw_image source 会静默失败）。当时这里持有
        # ``self._roi_img = image.Image(roi_w, roi_h, image.GRAYSCALE)``，
        # process() 末尾再 ``_roi_img.copy_from(src_wrap)`` 把 bin_inv_roi
        # 搬进 MMZ，每帧 ~48 KB memcpy ≈ 1-2 ms。
        #
        # 现在 OSD 已经改走 ``bytes(detection.binary_np)`` + Python 字节扫描，
        # 不再消费 binary_image，因此 _roi_img + copy_from 整条路径删掉，
        # 主路径每帧省 1-2 ms。binary_image 字段保留但置 None，向后兼容。

        # 预创建可复用的结果容器：bands 列表只在 process 里改字段，不重建。
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
            return result

        img_np = img.to_numpy_ref()

        # ---------- L0: cv_lite 二值化 ---------- #
        # bin_raw: 黑线像素 (val ≤ thresh) → 0；背景 (val > thresh) → 255
        try:
            bin_raw = cv_lite.grayscale_threshold_binary(
                self._image_shape, img_np, int(threshold), 255
            )
        except Exception as e:
            reraise_if_stop(e)
            print("[line_detector] L0 binarize failed:", e)
            self._reset_bands_invalid("L0_fail")
            return result

        # ---------- 反相到 ROI（黑线 = 前景 = 255） ---------- #
        # 仅对 ROI 行做反相，省一半 alloc / 算力。bin_inv_roi 是 heap-allocated
        # 的 ulab ndarray；L2 直接消费它，OSD 通过 ``bytes(bin_inv_roi)``
        # 物化后扫描，**不再走 image.Image MMZ 路径**——因此既不需要
        # _roi_img 也不需要 copy_from，主路径每帧省 1-2 ms。
        bin_inv_roi = 255 - bin_raw[self._roi_y : self._roi_y + self._roi_h, :]
        result.binary_np = bin_inv_roi  # 给 L2 / OSD bytes() 消费

        # ---------- L1: 形态学开运算（可选） ---------- #
        # ALLOC_REF wrap 让 image 模块"看到"ndarray buffer 做就地 open(1)，
        # 实测能正确写回 ulab 视角（image 模块对 ALLOC_REF 写入是 OK 的，
        # 仅作为 draw_image source / mask 才会静默失败）。L1 关闭时整块
        # wrap 都不创建，省一次 image.Image 构造。
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

        # ---------- L2: 5 条扫描带质心 ---------- #
        bands = result.bands
        prev_cx = -1.0
        prev_valid = False
        valid_count = 0
        mass_total = 0.0
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
            # **K230 ulab quirk（实测踩到）**：``np.sum(band_slice, axis=0)``
            # 在 K230 当前固件上**忽略 axis 参数**，对 (8, 320) 输入返回标量
            # 总和（等价 ``np.sum(band_slice)``）。下游 ``arange_x * col_sum``
            # 会被 broadcast 成 ``arange_x * scalar``，于是
            #   cx = sum(arange_x * scalar) / mass = scalar * sum(arange_x) / mass
            # 当 mass ≈ scalar 时（band_slice 求和后等于 mass），cx 退化为
            # ``sum(arange_x) = 0+1+...+(W-1) = 51040``（W=320），表现为 cxN
            # 永远卡在 51040.x 不变，且 width 也只剩 0/1（标量比较），从而触发
            # ``_draw_detection`` 把 51040 当 cx 喂给 OSD，display_x=127600
            # 屏幕外大坐标让 K230 OSD 进入慢路径 → FPS 从 33 跌到 13。
            #
            # 修复：手工按行累加 ``band_slice[r]``。每行是 (W,) 1-D ndarray，
            # 加法在 ulab 是向量化的；BAND_HEIGHT_PX=8 总共 7 次加法 ≈ 0.3ms，
            # 比 ``axis=0`` 求和稳但开销可忽略。
            col_sum = band_slice[0] + band_slice[1]
            for r in range(2, self._band_h):
                col_sum = col_sum + band_slice[r]

            # 防御：万一 ulab 行为再变，立即捕获。col_sum 必须是 W 长度向量；
            # 若是标量（len 抛 TypeError 或 != W），把整带置为无效并跳过。
            try:
                if len(col_sum) != self._roi_w:
                    band.reject = "col_sum_shape"
                    prev_valid = False
                    prev_cx = -1.0
                    continue
            except TypeError:
                band.reject = "col_sum_scalar"
                prev_valid = False
                prev_cx = -1.0
                continue

            mass = float(np.sum(col_sum))
            band.mass = mass
            mass_total += mass

            col_stats = None
            if self._prefer_bytes_col_stats:
                col_stats = _col_sum_stats_from_bytes(
                    col_sum, self._roi_w, self._col_sum_thr
                )

            # 等效宽度：col_sum 中超过阈值的列数。
            # ulab 的比较在不同固件上可能返回 bool(1) 或 uint8(255)；
            # 因此 sum 结果若超过 ROI 宽度，就退回 Python 计数（每带 320 次）。
            if col_stats is not None:
                band.width_px = col_stats[0]
            else:
                try:
                    width_mask = col_sum > self._col_sum_thr
                    band.width_px = int(np.sum(width_mask))
                    if band.width_px > self._roi_w:
                        col_stats = _col_sum_stats_from_bytes(
                            col_sum, self._roi_w, self._col_sum_thr
                        )
                        if col_stats is not None:
                            self._prefer_bytes_col_stats = True
                            band.width_px = col_stats[0]
                        else:
                            w = 0
                            thr = self._col_sum_thr
                            for v in col_sum:
                                if _scalar_to_int(v) > thr:
                                    w += 1
                            band.width_px = w
                except Exception:
                    col_stats = _col_sum_stats_from_bytes(
                        col_sum, self._roi_w, self._col_sum_thr
                    )
                    if col_stats is not None:
                        self._prefer_bytes_col_stats = True
                        band.width_px = col_stats[0]
                    else:
                        w = 0
                        thr = self._col_sum_thr
                        for v in col_sum:
                            if _scalar_to_int(v) > thr:
                                w += 1
                        band.width_px = w

            # 硬约束：mass / 宽度 / 与上一有效带的 Δcx
            if mass < self._min_mass[i]:
                band.reject = "mass<%d" % self._min_mass[i]
                prev_valid = False
                prev_cx = -1.0
                continue

            if col_stats is not None:
                cx = col_stats[1] / mass
            else:
                cx = float(np.sum(self._arange_x * col_sum) / mass)
            # K230 ulab 还可能在 1-D 加权求和里把 col_sum 当标量广播，
            # 表现为 cx≈sum(0..319)=51040。遇到越界值时退回按列加权求和。
            if cx < 0.0 or cx >= float(self._roi_w):
                col_stats = _col_sum_stats_from_bytes(
                    col_sum, self._roi_w, self._col_sum_thr
                )
                weighted = col_stats[1] if col_stats is not None else 0.0
                if col_stats is not None:
                    self._prefer_bytes_col_stats = True
                    band.width_px = col_stats[0]
                else:
                    x = 0
                    for v in col_sum:
                        weighted += x * _scalar_to_int(v)
                        x += 1
                cx = weighted / mass
            # cx 必须落在 ROI 列范围内；如果超出（理论上不可能，除非
            # col_sum 又变标量），把该带置为无效，避免下游 OSD ``draw_circle``
            # 拿到屏幕外大坐标进入 K230 image 模块慢路径，FPS 从 33 跌到 13。
            if cx < 0.0 or cx >= float(self._roi_w):
                band.reject = "cx_oob_%.1f" % cx
                prev_valid = False
                prev_cx = -1.0
                continue
            band.cx_px = cx

            w = band.width_px
            if w < self._w_min[i]:
                band.reject = "w<%d" % self._w_min[i]
                prev_valid = False
                prev_cx = -1.0
                continue
            if w > self._w_max[i]:
                band.reject = "w>%d" % self._w_max[i]
                prev_valid = False
                prev_cx = -1.0
                continue

            if prev_valid and prev_cx >= 0:
                dcx = cx - prev_cx
                if dcx < 0:
                    dcx = -dcx
                if dcx > self._delta_cx_max:
                    band.reject = "dcx>%d" % self._delta_cx_max
                    prev_valid = False
                    prev_cx = -1.0
                    continue

            band.valid = True
            valid_count += 1
            prev_valid = True
            prev_cx = cx

        result.mass_total = mass_total
        result.n_valid = valid_count

        # 近带在 bands[-1]（y_top 最大），远带在 bands[0]（y_top 最小）
        if bands[-1].valid:
            result.cx_near_px = bands[-1].cx_px
        if bands[0].valid:
            result.cx_far_px = bands[0].cx_px

        result.q_l2 = compute_q_l2(result, self.cfg)
        return result

    # ------------------------------------------------------------------ #
    # 辅助
    # ------------------------------------------------------------------ #

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
        return True
