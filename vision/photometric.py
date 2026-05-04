"""光度自适应阈值（plan §5.3 + §6.6）。

阶段 B 职责：

1. **启动期 bootstrap**：连续采样 ``PHOTO_BOOTSTRAP_FRAMES`` 帧 ROI，
   累积 Otsu / 背景均值 / 背景标准差，按公式

       thr_fallback   = μ_bg − k·σ_bg
       line_threshold = (thr_otsu + thr_fallback) / 2

   生成 ``Photometric.threshold``。

2. **运行期漂移监测**：每 ``PHOTO_DRIFT_CHECK_INTERVAL_MS`` 取一次 ROI mean，
   若 ``|Δμ_bg| > PHOTO_DRIFT_TRIG_DELTA_MU`` 进入复算态——之后的
   ``PHOTO_BOOTSTRAP_FRAMES`` 次 ``update()`` 把样本累入累加器，攒齐再
   finalize，全程不阻塞主循环（plan §5.3 "不阻塞主循环，放在低优先级任务"）。

设计要点：

- Otsu 直接调 ``image.histogram.get_threshold().value()``，K230 模块
  原生支持，不在 Python 层自写直方图遍历（plan §9.2 守则 3）。
- 直方图与统计量都只在 ``ROI_TOTAL_PX`` 区域上计算，避开远处天空 / 车体
  投影对全图直方图的污染（plan §4.3）。
- ``threshold`` 在 bootstrap 完成前先用 ``LINE_THRESHOLD_INIT`` 兜底，
  确保 detector 不会拿到 ``None``。
"""

import time

import config
from vision.interrupts import reraise_if_stop


class Photometric:
    """阈值与漂移监测的状态机。

    使用模式（参见 ``vision_line_tracking.py``）::

        photo = Photometric()
        photo.bootstrap(camera)          # 在 camera.start() 之后、主 while 之前
        while True:
            img = camera.read_algo_frame()
            photo.update(img)            # 周期性 + 漂移触发的渐进复算
            detector.process(img, photo.threshold)

    主要属性：

    - ``threshold`` (int)         当前 line_threshold（0~255）
    - ``mu_bg`` (float)           上次 finalize 的背景均值
    - ``sigma_bg`` (float)        上次 finalize 的背景标准差
    - ``thr_otsu`` (int)          上次 finalize 的 Otsu 阈值
    - ``is_recalibrating`` (bool) 当前是否处于漂移触发的累积复算
    - ``recalib_counter`` (int)   已累积的复算样本数
    - ``last_drift_delta`` (float) 上次触发漂移时的 |Δμ_bg|，用于诊断
    """

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else config

        self.threshold = self.cfg.LINE_THRESHOLD_INIT
        self.thr_otsu = self.cfg.LINE_THRESHOLD_INIT
        self.mu_bg = 0.0
        self.sigma_bg = 0.0
        self.last_drift_delta = 0.0

        self._last_check_ms = 0
        self._mu_bg_check = self.mu_bg

        self.is_recalibrating = False
        self.recalib_counter = 0
        self._sum_otsu = 0.0
        self._sum_mean = 0.0
        self._sum_std = 0.0

        self._roi = self.cfg.ROI_TOTAL_PX
        self._hist_bins = self.cfg.PHOTO_HIST_BINS

    # ------------------------------------------------------------------ #
    # 公共接口
    # ------------------------------------------------------------------ #
    def bootstrap(self, camera, n_frames=None):
        """启动期阻塞采样。返回 ``True`` 表示完成且阈值已更新。

        :param camera: 已 ``start()`` 的 :class:`vision.camera.Camera` 实例
        :param n_frames: 采样帧数，默认 ``config.PHOTO_BOOTSTRAP_FRAMES``。
        """
        if n_frames is None:
            n_frames = self.cfg.PHOTO_BOOTSTRAP_FRAMES
        if n_frames <= 0:
            return False

        self._reset_recalib_buffer()
        # 给 3 倍余量，避免极少数 snapshot 失败导致 bootstrap 永不结束。
        budget = n_frames * 3
        while self.recalib_counter < n_frames and budget > 0:
            budget -= 1
            img = camera.read_algo_frame()
            if img is None:
                continue
            self._accumulate_one(img)
            # 在下一次 snapshot 前释放本帧 Image 引用，避免 CHN VB buffer 被占住。
            img = None

        if self.recalib_counter == 0:
            print("[photometric] bootstrap FAILED: no frames collected")
            self.is_recalibrating = False
            return False

        n = self.recalib_counter
        self._finalize_recalib(n)
        self._last_check_ms = time.ticks_ms()
        self._mu_bg_check = self.mu_bg
        print(
            "[photometric] bootstrap done  frames=%d  "
            "mu_bg=%.1f  sigma_bg=%.1f  thr_otsu=%d  threshold=%d"
            % (n, self.mu_bg, self.sigma_bg, self.thr_otsu, self.threshold)
        )
        return True

    def update(self, img, now_ms=None):
        """运行期入口。返回 ``True`` 表示本帧导致 ``threshold`` 变化。

        语义：

        - **复算态**：本帧累入累加器；收齐 ``PHOTO_BOOTSTRAP_FRAMES`` 帧 finalize。
        - **平稳态**：每 ``PHOTO_DRIFT_CHECK_INTERVAL_MS`` 取一次 ROI mean，
          若漂移超阈值进入复算态。

        ``now_ms`` 默认 ``time.ticks_ms()``；外部已经取过 ticks 时传入避免重复。
        """
        if img is None:
            return False
        if now_ms is None:
            now_ms = time.ticks_ms()

        if self.is_recalibrating:
            self._accumulate_one(img)
            if self.recalib_counter >= self.cfg.PHOTO_BOOTSTRAP_FRAMES:
                n = self.recalib_counter
                self._finalize_recalib(n)
                self._last_check_ms = now_ms
                self._mu_bg_check = self.mu_bg
                print(
                    "[photometric] drift recalib done  frames=%d  "
                    "mu_bg=%.1f  sigma_bg=%.1f  thr_otsu=%d  threshold=%d"
                    % (n, self.mu_bg, self.sigma_bg, self.thr_otsu, self.threshold)
                )
                return True
            return False

        elapsed = time.ticks_diff(now_ms, self._last_check_ms)
        if elapsed < self.cfg.PHOTO_DRIFT_CHECK_INTERVAL_MS:
            return False
        self._last_check_ms = now_ms

        try:
            stats = img.get_statistics(roi=self._roi)
            mu_now = float(stats.mean())
        except Exception as e:
            reraise_if_stop(e)
            print("[photometric] drift check failed:", e)
            return False

        delta = mu_now - self._mu_bg_check
        if delta < 0.0:
            delta = -delta
        if delta > self.cfg.PHOTO_DRIFT_TRIG_DELTA_MU:
            self.last_drift_delta = delta
            print(
                "[photometric] drift trigger  mu_bg %.1f -> %.1f  (delta=%.1f)"
                % (self._mu_bg_check, mu_now, delta)
            )
            self._reset_recalib_buffer()
        else:
            self._mu_bg_check = mu_now
        return False

    def calibrate_blocking(self, camera, n_frames=None):
        """与 :meth:`bootstrap` 等价；提供给独立标定脚本调用，语义更明确。"""
        return self.bootstrap(camera, n_frames=n_frames)

    # ------------------------------------------------------------------ #
    # 内部
    # ------------------------------------------------------------------ #
    def _reset_recalib_buffer(self):
        self._sum_otsu = 0.0
        self._sum_mean = 0.0
        self._sum_std = 0.0
        self.recalib_counter = 0
        self.is_recalibrating = True

    def _accumulate_one(self, img):
        """把当前帧的 ROI Otsu / mean / std 累入累加器。"""
        try:
            hist = img.get_histogram(roi=self._roi, bins=self._hist_bins)
            stats = hist.get_statistics()
            thr_obj = hist.get_threshold()
            otsu = thr_obj.value()
        except Exception as e:
            reraise_if_stop(e)
            print("[photometric] sample failed:", e)
            return
        self._sum_otsu += otsu
        self._sum_mean += float(stats.mean())
        self._sum_std += float(stats.stdev())
        self.recalib_counter += 1

    def _finalize_recalib(self, n):
        """合成最终阈值。

        plan §5.3 的公式 ``thr = (Otsu + (μ−kσ)) / 2`` 假设背景近似高斯——
        当 σ 接近或超过 μ（典型情形：桌面杂物 / 远景灯具 / 复杂阴影），
        ``μ−kσ`` 会落到 0 以下；早期实现把它钳到 0 后照样参与平均，结果是
        把 Otsu 阈值人为拉低一半，黑色容差锐减。

        修复策略：
        - fallback 只有在 ``0 < μ−kσ < Otsu`` 时才视作有效信号参与平均；
          否则**只信 Otsu**，并打印一次诊断日志方便排障。
        - 最后再叠加 ``LINE_THRESHOLD_BIAS`` 偏置，并钳到 ``[MIN, MAX]``。
          这样允许在不修算法的前提下，按"电工胶带 vs 哑光黑塑料"等场景
          做粗调。
        """
        if n <= 0:
            self.is_recalibrating = False
            return
        self.thr_otsu = int(round(self._sum_otsu / n))
        self.mu_bg = self._sum_mean / n
        self.sigma_bg = self._sum_std / n

        thr_fb = self.mu_bg - self.cfg.PHOTO_FALLBACK_K_SIGMA * self.sigma_bg
        # fallback 公式假设近似高斯背景；σ 过大时该假设失效，直接退化为 Otsu。
        if thr_fb <= 0.0 or thr_fb >= 254.0 or thr_fb >= self.thr_otsu:
            new_thresh_f = float(self.thr_otsu)
            fb_used = False
        else:
            new_thresh_f = (self.thr_otsu + thr_fb) / 2.0
            fb_used = True

        bias = float(self.cfg.get("LINE_THRESHOLD_BIAS", 0))
        if bias != 0.0:
            new_thresh_f += bias

        lo = int(self.cfg.get("LINE_THRESHOLD_MIN", 1))
        hi = int(self.cfg.get("LINE_THRESHOLD_MAX", 254))
        if lo < 1:
            lo = 1
        if hi > 254:
            hi = 254
        new_thresh = int(round(new_thresh_f))
        if new_thresh < lo:
            new_thresh = lo
        elif new_thresh > hi:
            new_thresh = hi

        if not fb_used:
            print(
                "[photometric] fallback rejected (mu=%.1f sigma=%.1f "
                "thr_fb=%.1f) -> use Otsu only"
                % (self.mu_bg, self.sigma_bg, thr_fb)
            )

        self.threshold = new_thresh
        self.is_recalibrating = False

    # ------------------------------------------------------------------ #
    # 自检（plan §11.2 强制项）
    # ------------------------------------------------------------------ #
    def self_test(self):
        """检查关键参数在合理区间。"""
        if not (1 <= self.threshold <= 254):
            print("[photometric.self_test] threshold out of range:", self.threshold)
            return False
        if self.cfg.PHOTO_BOOTSTRAP_FRAMES <= 0:
            print("[photometric.self_test] bootstrap_frames must be > 0")
            return False
        if self.cfg.PHOTO_DRIFT_TRIG_DELTA_MU <= 0:
            print("[photometric.self_test] drift_trig must be > 0")
            return False
        return True
