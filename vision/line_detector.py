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
            # K230 ulab 的 ndarray 没有 ``.sum()`` 实例方法（只在新版 ulab 上才有），
            # 这里**统一使用函数式** ``np.sum(arr, axis=...)``，避免
            # AttributeError: 'ndarray' object has no attribute 'sum'。
            col_sum = np.sum(band_slice, axis=0)
            mass = float(np.sum(col_sum))
            band.mass = mass
            mass_total += mass

            # 等效宽度：col_sum 中超过阈值的列数。
            # ulab 的比较返回 uint8/bool ndarray；同样走函数式 sum；
            # 若该 ulab 版本不支持 bool ndarray 求和，退回 Python 计数（每带 320 次）。
            try:
                width_mask = col_sum > self._col_sum_thr
                band.width_px = int(np.sum(width_mask))
            except Exception:
                w = 0
                thr = self._col_sum_thr
                for v in col_sum:
                    if v > thr:
                        w += 1
                band.width_px = w

            # 硬约束：mass / 宽度 / 与上一有效带的 Δcx
            if mass < self._min_mass[i]:
                band.reject = "mass<%d" % self._min_mass[i]
                prev_valid = False
                prev_cx = -1.0
                continue

            cx = float(np.sum(self._arange_x * col_sum) / mass)
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
