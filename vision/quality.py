"""视觉质量评分（plan §6.6）。

阶段 B 落地 ``compute_q_l2``：仅含 L2 相关分量（mass + 连续性 + 有效带数）。
阶段 C 新增 ``compute_q_full``：在 L2 子项基础上叠加 IPM RANSAC 的内点率
``geom`` 子项 + 半径先验 ``r_prior`` 子项；两者共存（阶段 B bench 模式可
继续只看 Q_L2，阶段 C 起主入口默认输出 Q_full）。

公式（与 ``config.Q_L2_*`` / ``config.Q_W_*`` 对应）::

    # 阶段 B：仅 L2 子集
    Q_L2 = w_mass  · sat(mass_total / Q_L2_MASS_NOMINAL_TOTAL, 0, 1) · 100
         + w_cont  · sat(1 − jitter_cx / Q_L2_JITTER_REF_PX, 0, 1)   · 100   (n_valid ≥ 2)
         + w_valid · (n_valid / BAND_COUNT)                          · 100

    # 阶段 C：plan §6.6 完整版
    Q_full = w_mass    · sat(mass_total / Q_L2_MASS_NOMINAL_TOTAL, 0, 1) · 100
           + w_geom    · sat(arc.inlier_count / max(arc.sample_count, 1), 0, 1) · 100
           + w_cont    · sat(1 − jitter_cx / Q_L2_JITTER_REF_PX, 0, 1)   · 100
           + w_r_prior · sat(1 − |R̂ − R_PRIOR| / Q_R_PRIOR_NORM_MM, 0, 1) · 100

    权重（``Q_W_*``）总和 = 1.0；plan §6.6 推荐 0.3 / 0.3 / 0.2 / 0.2。

其中 ``jitter_cx`` 取相邻 *有效* 带 cx 的最大 |Δcx|。

**重要**：``n_valid < 2`` 时 ``q_cont`` **强制为 0** 而不是"jitter=0 视为
完美连续"。否则 V=0/5（完全失锁）会被打成 Q=80（mass 50 + cont 30 + valid 0），
误导分级（"good" = 全速行驶），实测会引发 hold/lost 误判。

**Q_full 的 arc 缺失行为**：当 ``arc is None`` 或 ``arc.succeeded=False``
时，``q_geom`` 与 ``q_r_prior`` 都强制为 0。这种"几何失败但 L2 还在"的
工况下，Q_full 上限 = w_mass·100 + w_cont·100 = 50（按推荐权重），自然
落到 lost 分级（plan §7.2 ``Q < 40`` 视为丢线）；但实际工程里 ``compute_q_l2``
仍然返回纯 L2 评分，由调用方决定输出 Q_L2 还是 Q_full。

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

    # ---- 连续性项：相邻有效带 cx 的最大 |Δcx|（与 Q_full 共享实现）----
    # n_valid < 2 时 _q_jitter 强制返回 0；不再走"jitter=0 视为完美连续"
    # 路径——那条路径会让 V=0/5（完全失锁）打出 Q=80（=50+30+0），触发
    # grade()="good" 把上层控制误导为"全速行驶"。
    q_cont = _q_jitter(detection, float(cfg.Q_L2_JITTER_REF_PX))

    # ---- 有效带占比项 ----
    q_valid = (detection.n_valid / float(band_count)) * 100.0

    q = (
        cfg.Q_L2_W_MASS * q_mass
        + cfg.Q_L2_W_CONT * q_cont
        + cfg.Q_L2_W_VALID * q_valid
    )
    return _saturate(q, 0.0, 100.0)


def _q_jitter(detection, jitter_ref):
    """共享给 Q_L2 / Q_full 的连续性分量计算。"""
    if detection.n_valid < 2:
        return 0.0
    bands = detection.bands
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
    if jitter_ref <= 0:
        return 0.0
    return _saturate(1.0 - jitter / jitter_ref, 0.0, 1.0) * 100.0


def compute_q_full(detection, arc, cfg=None):
    """plan §6.6 完整版 Q（mass + geom + cont + r_prior）。

    :param detection: :class:`vision.line_detector.DetectionResult`
    :param arc: :class:`vision.geometry.ArcResult` 或 ``None``
        （``None`` / ``arc.succeeded=False`` 时 ``q_geom`` 与 ``q_r_prior``
        都置 0，自然降级到 lost 分级）
    :param cfg: ``config`` 模块或兼容对象；默认全局 ``config``。
    :return: float 0~100，已经裁剪到合法区间。
    """
    if cfg is None:
        cfg = config

    bands = detection.bands
    band_count = len(bands)
    if band_count == 0:
        return 0.0

    # ---- mass 项（与 Q_L2 共享）----
    mass_nominal = float(cfg.Q_L2_MASS_NOMINAL_TOTAL)
    if mass_nominal <= 0:
        q_mass = 0.0
    else:
        q_mass = _saturate(detection.mass_total / mass_nominal, 0.0, 1.0) * 100.0

    # ---- geom 项：RANSAC 内点率 ----
    if arc is None or not arc.succeeded or arc.sample_count <= 0:
        q_geom = 0.0
    else:
        q_geom = _saturate(arc.inlier_count / float(arc.sample_count), 0.0, 1.0) * 100.0

    # ---- 连续性项 ----
    q_cont = _q_jitter(detection, float(cfg.Q_L2_JITTER_REF_PX))

    # ---- r_prior 项：|R̂ − R_PRIOR| 偏差 ----
    if arc is None or not arc.succeeded:
        q_r_prior = 0.0
    else:
        norm_mm = float(cfg.Q_R_PRIOR_NORM_MM)
        if norm_mm <= 0:
            q_r_prior = 0.0
        else:
            dev = arc.R - float(cfg.R_PRIOR_MM)
            if dev < 0.0:
                dev = -dev
            q_r_prior = _saturate(1.0 - dev / norm_mm, 0.0, 1.0) * 100.0

    q = (
        cfg.Q_W_MASS * q_mass
        + cfg.Q_W_GEOM * q_geom
        + cfg.Q_W_CONT * q_cont
        + cfg.Q_W_R_PRIOR * q_r_prior
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
