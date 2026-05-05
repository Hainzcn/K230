"""视觉质量评分（plan §6.6）。

阶段 B 仅含 L2 相关分量（mass + 连续性 + 有效带数）；阶段 C 完成 IPM + RANSAC
后再接入 ``geom`` 与 ``r_prior`` 子项。

公式（与 ``config.Q_L2_*`` 对应）::

    Q_L2 = w_mass  * sat(mass_total / Q_L2_MASS_NOMINAL_TOTAL, 0, 1) * 100
         + w_cont  * (1 − jitter_cx / Q_L2_JITTER_REF_PX)            * 100   (n_valid ≥ 2)
         + w_valid * (n_valid / BAND_COUNT)                          * 100

其中 ``jitter_cx`` 取相邻 *有效* 带 cx 的最大 |Δcx|。

**重要**：``n_valid < 2`` 时 ``q_cont`` **强制为 0** 而不是"jitter=0 视为
完美连续"。否则 V=0/5（完全失锁）会被打成 Q=80（mass 50 + cont 30 + valid 0），
误导分级（"good" = 全速行驶），实测会引发 hold/lost 误判。

设计：纯函数 + 无副作用，便于阶段 C/E 扩展时只增不改。``DetectionResult``
的 ``mass_total`` 与 ``n_valid`` 在 detector 写入后才调用本函数，调用方不需要
做空值保护。
"""

import config


def _saturate(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def compute_q_l2(detection, cfg=None):
    """计算 ``detection`` 的 L2 质量评分。

    :param detection: :class:`vision.line_detector.DetectionResult`
    :param cfg: ``config`` 模块或兼容对象；默认全局 ``config``。
    :return: float 0~100，已经裁剪到合法区间。
    """
    if cfg is None:
        cfg = config

    bands = detection.bands
    band_count = len(bands)
    if band_count == 0:
        return 0.0

    # ---- mass 项 ----
    mass_nominal = float(cfg.Q_L2_MASS_NOMINAL_TOTAL)
    if mass_nominal <= 0:
        q_mass = 0.0
    else:
        q_mass = _saturate(detection.mass_total / mass_nominal, 0.0, 1.0) * 100.0

    # ---- 连续性项：相邻有效带 cx 的最大 |Δcx| ----
    # n_valid < 2 时无法定义"相邻 cx 跳变"，强制 q_cont=0；不再走"jitter=0
    # 视为完美连续"路径——那条路径会让 V=0/5（完全失锁）打出 Q=80（=50+30+0），
    # 触发 grade()="good" 把上层控制误导为"全速行驶"。
    if detection.n_valid < 2:
        q_cont = 0.0
    else:
        jitter = 0.0
        prev_cx = None
        for b in bands:
            if not b.valid:
                prev_cx = None
                continue
            if prev_cx is not None:
                d = b.cx_px - prev_cx
                if d < 0.0:
                    d = -d
                if d > jitter:
                    jitter = d
            prev_cx = b.cx_px
        jitter_ref = float(cfg.Q_L2_JITTER_REF_PX)
        if jitter_ref <= 0:
            q_cont = 0.0
        else:
            q_cont = _saturate(1.0 - jitter / jitter_ref, 0.0, 1.0) * 100.0

    # ---- 有效带占比项 ----
    q_valid = (detection.n_valid / float(band_count)) * 100.0

    q = (
        cfg.Q_L2_W_MASS * q_mass
        + cfg.Q_L2_W_CONT * q_cont
        + cfg.Q_L2_W_VALID * q_valid
    )
    return _saturate(q, 0.0, 100.0)


def grade(q, cfg=None):
    """把 Q 值映射为 plan §6.6/§7.2 分级标签，便于日志与 OSD 用色。

    返回 ``"good"`` / ``"degrade"`` / ``"hold"`` / ``"lost"``。
    """
    if cfg is None:
        cfg = config
    if q >= cfg.Q_GOOD:
        return "good"
    if q >= cfg.Q_DEGRADE:
        return "degrade"
    if q >= cfg.Q_HOLD:
        return "hold"
    return "lost"
