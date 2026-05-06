"""时域估计器（plan §7.4）。

本阶段只落地一阶 EMA + 符号防抖；一维 Kalman 留给阶段 E 之后（plan §7.4
明确写"阶段 E 之后再升级"）。

接口约定
=========

- ``EmaEstimator(alpha, age_max)``：单标量状态。
- ``update(x, valid) -> (current, age)``：``valid=True`` 喂入新观测，
  ``valid=False`` 仅老化（``age += 1``，但保持上一帧 ``current``）；
  ``age >= age_max`` 时自动 ``reset()``。
- ``current`` / ``age`` / ``has_value`` 三个只读属性。
- ``reset()``：清状态。

为什么 ``e_y`` / ``ψ_e`` 各自一个 EMA 实例
========================================

plan §7.4 明确"滤波器只作用在 e_y, ψ_e，不滤波 Q"——分量级 EMA 比向量
级更直观，参数（α）独立调，调试时也能单独看每路收敛性能。``Q`` 一旦滞后
就丧失及时降级能力（plan §7.4 原话）。

符号防抖（plan §8.1）
=====================

``SignDebounce`` 独立维护，不与 ``EmaEstimator`` 强绑定：调用方先把原始
观测 ``x_raw`` 喂给 ``SignDebounce.filter(x_raw)``，得到"已防抖的有符号
观测"，再喂给 EMA。这样防抖与平滑解耦，单测更容易。

防抖逻辑：

- ``sign(x_raw)`` 与上次接受的符号一致 → 直接放行；
- ``sign(x_raw)`` 翻转 → 必须连续 ``debounce_frames`` 帧同方向才接受新符号；
  期间所有这些观测都被强制翻号（保留上次符号，使用其 magnitude）。
- 这样可吸收单帧抖动诱发的方向反转（例如 light flicker 导致 NEAR cx
  瞬间偏到对侧）。
"""

import config


class SignDebounce:
    """plan §8.1 sign 防抖：连续 ``debounce_frames`` 帧反号才接受翻转。"""

    __slots__ = ("debounce_frames", "_last_accepted_sign", "_pending_sign", "_pending_count")

    def __init__(self, debounce_frames=None):
        if debounce_frames is None:
            debounce_frames = int(config.SIGN_FLIP_DEBOUNCE_FRAMES)
        self.debounce_frames = max(1, int(debounce_frames))
        self._last_accepted_sign = 0     # 0=未初始化, ±1
        self._pending_sign = 0
        self._pending_count = 0

    def reset(self):
        self._last_accepted_sign = 0
        self._pending_sign = 0
        self._pending_count = 0

    def filter(self, x):
        """返回防抖后的有符号 ``x``。

        - 第一次调用直接接受 sign(x) 作为基线，``x`` 不动；
        - 后续调用若 ``sign(x)`` 与基线一致则直接放行（清空 pending）；
        - 若 ``sign(x)`` 与基线相反，pending 计数 +1；只有连续 ``debounce_frames``
          帧都反号才把 pending sign 接受为新基线，并放行新观测；
        - 期间所有"反号但未达计数"的观测都被强制翻号成基线方向（magnitude 不变）。

        ``x == 0``：视作"无方向信息"，直接放行（不增不减 pending）。
        """
        if x > 0.0:
            sgn = 1
        elif x < 0.0:
            sgn = -1
        else:
            return x

        if self._last_accepted_sign == 0:
            self._last_accepted_sign = sgn
            self._pending_sign = 0
            self._pending_count = 0
            return x

        if sgn == self._last_accepted_sign:
            self._pending_sign = 0
            self._pending_count = 0
            return x

        # 反号
        if sgn == self._pending_sign:
            self._pending_count += 1
        else:
            self._pending_sign = sgn
            self._pending_count = 1

        if self._pending_count >= self.debounce_frames:
            self._last_accepted_sign = sgn
            self._pending_sign = 0
            self._pending_count = 0
            return x

        # 强制翻号：保留 magnitude，符号回到上次接受值
        return -x


class EmaEstimator:
    """一阶 EMA 平滑器；带 valid 失效衰减/自动复位。

    更新规则::

        valid=True:
            if not has_value:    current = x;  age = 0
            else:                current = α·x + (1−α)·current;  age = 0
        valid=False:
            age += 1
            if age >= age_max:   reset()

    α 越大越跟随、越小越平滑；plan §7.4 推荐 0.4~0.6。
    """

    __slots__ = ("alpha", "age_max", "_current", "_age", "_has_value")

    def __init__(self, alpha=0.5, age_max=None):
        self.alpha = float(alpha)
        if age_max is None:
            age_max = int(config.EMA_AGE_MAX_FRAMES)
        self.age_max = max(1, int(age_max))
        self._current = 0.0
        self._age = 0
        self._has_value = False

    @property
    def current(self):
        return self._current

    @property
    def age(self):
        return self._age

    @property
    def has_value(self):
        return self._has_value

    def reset(self):
        self._current = 0.0
        self._age = 0
        self._has_value = False

    def update(self, x, valid=True):
        """喂一帧观测；返回 ``(current, age)``。"""
        if valid:
            if self._has_value:
                self._current = self.alpha * float(x) + (1.0 - self.alpha) * self._current
            else:
                self._current = float(x)
                self._has_value = True
            self._age = 0
        else:
            if self._has_value:
                self._age += 1
                if self._age >= self.age_max:
                    self.reset()
        return self._current, self._age


class PathErrorEstimator:
    """打包 ``e_y_mm`` / ``ψ_e_mrad`` 两路 EMA + 各自符号防抖。

    主入口只持有一个实例即可；详见 ``vision_line_tracking.py``。
    """

    __slots__ = ("ema_e_y", "ema_psi_e", "deb_e_y", "deb_psi_e")

    def __init__(self, cfg=None):
        if cfg is None:
            cfg = config
        self.ema_e_y = EmaEstimator(
            alpha=float(cfg.EMA_ALPHA_E_Y),
            age_max=int(cfg.EMA_AGE_MAX_FRAMES),
        )
        self.ema_psi_e = EmaEstimator(
            alpha=float(cfg.EMA_ALPHA_PSI),
            age_max=int(cfg.EMA_AGE_MAX_FRAMES),
        )
        self.deb_e_y = SignDebounce(int(cfg.SIGN_FLIP_DEBOUNCE_FRAMES))
        self.deb_psi_e = SignDebounce(int(cfg.SIGN_FLIP_DEBOUNCE_FRAMES))

    def reset(self):
        self.ema_e_y.reset()
        self.ema_psi_e.reset()
        self.deb_e_y.reset()
        self.deb_psi_e.reset()

    def update(self, e_y_raw, psi_e_raw, valid):
        """返回 ``(e_y_filt, psi_e_filt, e_y_age, psi_e_age)``。

        ``valid=False`` 时不喂新观测、只让 EMA 老化；防抖状态不动（避免
        丢线后 sign 重置带来的"恢复时第一帧又被吸收"漏报）。
        """
        if valid:
            e_y_d = self.deb_e_y.filter(e_y_raw)
            psi_e_d = self.deb_psi_e.filter(psi_e_raw)
            ey, age_e = self.ema_e_y.update(e_y_d, True)
            psi, age_p = self.ema_psi_e.update(psi_e_d, True)
        else:
            ey, age_e = self.ema_e_y.update(0.0, False)
            psi, age_p = self.ema_psi_e.update(0.0, False)
        return ey, psi, age_e, age_p

    @property
    def has_value(self):
        return self.ema_e_y.has_value or self.ema_psi_e.has_value


__all__ = ["EmaEstimator", "SignDebounce", "PathErrorEstimator"]
