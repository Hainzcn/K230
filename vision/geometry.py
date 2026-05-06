"""阶段 C 几何拟合（plan §6.3）。

输入：地面坐标系（mm）下的若干 ``(x_g, y_g)`` 点（来自
``vision.ground_mapper.GroundMapper.bands_to_ground``）。

输出：

- ``ArcResult``：RANSAC 圆弧 (xc, yc, R)。``succeeded=False`` 时调用方走
  L3b 直线回归 fallback。
- ``LineResult``：总最小二乘 (TLS) 直线（含切线、法线、质心、残差），用于
  直道段 / RANSAC 失败时的兜底。
- ``compute_path_errors_arc(arc)`` / ``compute_path_errors_line(line)``：
  把几何拟合结果换算为 plan §1.3 契约下的 ``(e_y_mm, psi_e_mrad)``。

符号约定（plan §1.3 + §7.1 控制律自洽性）::

    e_y > 0  ⇔  黑线在车的右方 (车体 y 轴负向)；ω_fb = k_y · e_y → 右转纠正
    ψ_e > 0  ⇔  切线方向在车头的右方；ω_fb += k_ψ · ψ_e → 右转纠正

计算细节
=========

**RANSAC 圆弧**：

5 个候选点 → C(5,3)=10 个三元组，全部枚举（plan §6.3 给的 "迭代 20~40 次"
对随机采样，N=5 时反而枚举更省）。对每个三元组：

1. 用代数公式求过 3 点的圆 (xc, yc, R)；3 点共线时跳过；
2. 校验半径先验 ``|R − R_PRIOR| ≤ R_PRIOR_TOL_MM``；不满足直接丢弃；
3. 统计内点数 ``inliers = #{ p_i : |‖p_i − (xc, yc)‖ − R| ≤ ε }``；
4. 按 ``(inliers desc, |R−R_PRIOR| asc)`` 排序选最优。

返回的 ``ArcResult`` 携带 ``succeeded=True`` 仅当：
``best_inliers >= RANSAC_MIN_INLIERS``。

**TLS 直线**：

主方向角 ``φ = 0.5 · atan2(2 Sxy, Sxx − Syy)``；切线 = (cos φ, sin φ)，
法线 = (−sin φ, cos φ)。强制法线朝车体 +y（左），切线朝车头 +x（前），
统一符号约定。

**路径误差换算**（关键的符号约定都在这里实现）：见
``compute_path_errors_arc`` / ``_line`` 的 docstring。

K230 性能注意
=============

全部走纯 Python 标量运算（``math`` + 标量加减乘除）。**不引入 ulab**：
阶段 B task_log §4 记录了 ulab 在 K230 上的多个 quirk（``np.sum(axis=...)``
失效、加权质心 1-D 广播成全列和），geometry 路径不大（≤10 个三元组 × 几
十次乘加），用纯 Python 不会成为性能瓶颈，可移植性更好。
"""

import math

import config


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------


class ArcResult:
    """RANSAC 圆弧拟合结果。"""

    __slots__ = (
        "xc", "yc", "R",
        "inlier_count", "sample_count",
        "succeeded",
        "mode",          # "ransac" / "none"
        "r_prior_dev",   # |R − R_PRIOR|
    )

    def __init__(self):
        self.xc = 0.0
        self.yc = 0.0
        self.R = 0.0
        self.inlier_count = 0
        self.sample_count = 0
        self.succeeded = False
        self.mode = "none"
        self.r_prior_dev = float("inf")


class LineResult:
    """总最小二乘 (TLS) 直线拟合结果。"""

    __slots__ = (
        "cx", "cy",          # 质心（地面坐标 mm）
        "tx", "ty",          # 切线方向单位向量（朝车头 +x）
        "nx", "ny",          # 法线方向单位向量（朝车体 +y）
        "residual_std",      # 法向残差标准差 mm
        "sample_count",
        "succeeded",
        "mode",              # "lsq" / "none"
    )

    def __init__(self):
        self.cx = 0.0
        self.cy = 0.0
        self.tx = 1.0
        self.ty = 0.0
        self.nx = 0.0
        self.ny = 1.0
        self.residual_std = 0.0
        self.sample_count = 0
        self.succeeded = False
        self.mode = "none"


# ---------------------------------------------------------------------------
# 圆拟合：3 点代数解
# ---------------------------------------------------------------------------


def fit_circle_3pt(x1, y1, x2, y2, x3, y3):
    """过 3 点的圆，代数解。共线时返回 ``None``。

    用一般式 ``x² + y² + D x + E y + F = 0``；3 点代入解 (D, E, F)，
    然后 ``xc = -D/2``、``yc = -E/2``、``R² = xc² + yc² − F``。
    """
    a1 = x1 - x2
    b1 = y1 - y2
    c1 = (x1 * x1 + y1 * y1) - (x2 * x2 + y2 * y2)
    a2 = x1 - x3
    b2 = y1 - y3
    c2 = (x1 * x1 + y1 * y1) - (x3 * x3 + y3 * y3)
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:
        return None
    inv2 = 0.5 / det
    xc = (c1 * b2 - c2 * b1) * inv2
    yc = (a1 * c2 - a2 * c1) * inv2
    dx = x1 - xc
    dy = y1 - yc
    R2 = dx * dx + dy * dy
    if R2 < 0.0:
        return None
    return xc, yc, math.sqrt(R2)


def fit_circle_ransac(points, cfg=None):
    """枚举式 RANSAC 圆拟合（plan §6.3）。

    :param points: list of (x_mm, y_mm)，建议 ≥ 3 个；通常是
        :class:`vision.ground_mapper.GroundPoint` 的有效点。
    :param cfg: ``config`` 模块。
    :return: :class:`ArcResult`。``succeeded=False`` 时调用方走
        ``fit_line_lsq`` + ``compute_path_errors_line``。
    """
    if cfg is None:
        cfg = config
    arc = ArcResult()
    arc.sample_count = len(points)
    if arc.sample_count < cfg.RANSAC_MIN_SAMPLES:
        return arc

    r_prior = float(cfg.R_PRIOR_MM)
    r_tol = float(cfg.R_PRIOR_TOL_MM)
    eps = float(cfg.RANSAC_INLIER_EPS_MM)
    min_inliers = int(cfg.RANSAC_MIN_INLIERS)

    n = arc.sample_count
    best_inliers = -1
    best_dev = float("inf")
    best_xc = 0.0
    best_yc = 0.0
    best_R = 0.0

    # 枚举所有 C(n, 3) 三元组
    for i in range(n - 2):
        x1, y1 = points[i]
        for j in range(i + 1, n - 1):
            x2, y2 = points[j]
            for k in range(j + 1, n):
                x3, y3 = points[k]
                circle = fit_circle_3pt(x1, y1, x2, y2, x3, y3)
                if circle is None:
                    continue
                xc, yc, R = circle
                dev = R - r_prior
                if dev < 0.0:
                    dev = -dev
                if dev > r_tol:
                    continue
                # 统计 inliers
                inliers = 0
                for px, py in points:
                    dx = px - xc
                    dy = py - yc
                    err = math.sqrt(dx * dx + dy * dy) - R
                    if err < 0.0:
                        err = -err
                    if err <= eps:
                        inliers += 1
                # 选优：先看 inliers，再看 |R−R_prior|
                if (inliers > best_inliers
                        or (inliers == best_inliers and dev < best_dev)):
                    best_inliers = inliers
                    best_dev = dev
                    best_xc = xc
                    best_yc = yc
                    best_R = R

    if best_inliers >= min_inliers:
        arc.xc = best_xc
        arc.yc = best_yc
        arc.R = best_R
        arc.inlier_count = best_inliers
        arc.r_prior_dev = best_dev
        arc.succeeded = True
        arc.mode = "ransac"
    return arc


# ---------------------------------------------------------------------------
# 直线 LSQ
# ---------------------------------------------------------------------------


def fit_line_lsq(points):
    """总最小二乘 (TLS) 直线拟合。

    主方向用 2x2 协方差矩阵的最大特征向量给出（解析公式 ``φ = 0.5 ·
    atan2(2·Sxy, Sxx − Syy)``，无需迭代）。

    返回的 ``LineResult.tx, ty`` 朝车头 +x（``tx > 0``）；
    ``nx, ny`` 朝车体 +y（``ny > 0``）。``succeeded=False`` 表示样本不足
    或退化（所有点重合）。
    """
    line = LineResult()
    n = len(points)
    line.sample_count = n
    if n < 2:
        return line

    sx = 0.0
    sy = 0.0
    for px, py in points:
        sx += px
        sy += py
    cx = sx / n
    cy = sy / n

    sxx = 0.0
    syy = 0.0
    sxy = 0.0
    for px, py in points:
        dx = px - cx
        dy = py - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    if sxx + syy < 1e-9:
        return line

    phi = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    tx = cphi
    ty = sphi
    nx = -sphi
    ny = cphi

    # 法线朝 +y（车体左）；翻号时 tangent 不动。
    if ny < 0.0:
        nx = -nx
        ny = -ny
    # 切线朝 +x（车头前）。
    if tx < 0.0:
        tx = -tx
        ty = -ty

    # 残差（法向距离）
    sse = 0.0
    for px, py in points:
        dx = px - cx
        dy = py - cy
        d = dx * nx + dy * ny
        sse += d * d
    line.residual_std = math.sqrt(sse / n)

    line.cx = cx
    line.cy = cy
    line.tx = tx
    line.ty = ty
    line.nx = nx
    line.ny = ny
    line.succeeded = True
    line.mode = "lsq"
    return line


# ---------------------------------------------------------------------------
# 路径误差换算（plan §6.3 + §1.3 + §7.1 符号自洽）
# ---------------------------------------------------------------------------


def compute_path_errors_arc(arc, cfg=None):
    """从 RANSAC 圆弧推 ``(e_y_mm, psi_e_mrad)``。

    几何细节
    -------

    车体在地面坐标原点 ``P = (0, 0)``，车头沿 +x 方向；圆心
    ``C = (xc, yc)``、半径 ``R``。

    黑线最近点（车体到圆环的最短距离落点）::

        N = C + R · (P − C) / ‖P − C‖  =  C · (1 − R/d)

    符号约定（plan §1.3 / §7.1 自洽推导）::

        e_y > 0  ⇔  黑线在车的右方  ⇔  N_y < 0  ⇔  e_y_mm = −N_y
                 = −yc · (1 − R_prior/d) = yc · (R_prior/d − 1)

    切线方向（沿圆环前进，朝车头 +x 那一侧）::

        t = (sign(yc) · yc / d, −sign(yc) · xc / d) = (|yc|/d, −sign(yc)·xc/d)

    航向误差（车头偏右于切线 = 切线在车头右方 ⇔ ψ_e > 0）::

        ψ_e_rad = atan2(−t_y, t_x) = atan2(sign(yc) · xc, |yc|)

    退化与边界
    --------

    - ``arc.succeeded=False`` → 返回 ``(0.0, 0.0, valid=False)``；
    - 圆心几乎落在车上 ``d < 1 mm`` → 返回 invalid；
    - ``yc`` 极小（几乎为 0）→ 切线方向退化为 ±x，``ψ_e`` 取 0 即可（圆心
      在车的正前 / 正后方时车头几乎贴切线）。

    :return: ``(e_y_mm, psi_e_mrad, valid)`` 三元组。
    """
    if cfg is None:
        cfg = config
    if arc is None or not arc.succeeded:
        return 0.0, 0.0, False

    xc = arc.xc
    yc = arc.yc
    d2 = xc * xc + yc * yc
    if d2 < 1.0:        # < 1 mm² 视为圆心在车上
        return 0.0, 0.0, False
    d = math.sqrt(d2)
    r_prior = float(cfg.R_PRIOR_MM)

    # 横向偏差：黑线相对车的"+y body"方向距离取反 = 黑线在右为正
    e_y_mm = yc * (r_prior / d - 1.0)

    # 航向误差：sign(yc) · xc 为分子；当 yc=0 时退化为 0 (圆心在前后正方向上)
    if yc > 1e-6:
        sign_yc = 1.0
    elif yc < -1e-6:
        sign_yc = -1.0
    else:
        sign_yc = 0.0
    if sign_yc == 0.0:
        psi_e_rad = 0.0
    else:
        psi_e_rad = math.atan2(sign_yc * xc, abs(yc))
    psi_e_mrad = psi_e_rad * 1000.0
    return e_y_mm, psi_e_mrad, True


def compute_path_errors_line(line, cfg=None):
    """从 TLS 直线推 ``(e_y_mm, psi_e_mrad)``。

    直道段 / RANSAC 失败时的 fallback。等价于"圆心在无穷远的圆"。

    几何::

        signed_distance = (P − centroid) · normal = −(cx · nx + cy · ny)
                          (法线已强制朝 +y 车体；signed > 0 ⇒ 车在直线左侧)

    符号（与 ``compute_path_errors_arc`` 自洽）::

        e_y_mm = signed_distance      （车在直线左 ⇔ 黑线在车右 ⇔ e_y > 0 ✓）
        ψ_e_rad = atan2(−ty, tx)      （切线相对车头方向，正号偏右）
    """
    if cfg is None:
        cfg = config
    if line is None or not line.succeeded:
        return 0.0, 0.0, False

    e_y_mm = -(line.cx * line.nx + line.cy * line.ny)
    psi_e_rad = math.atan2(-line.ty, line.tx)
    psi_e_mrad = psi_e_rad * 1000.0
    return e_y_mm, psi_e_mrad, True


__all__ = [
    "ArcResult",
    "LineResult",
    "fit_circle_3pt",
    "fit_circle_ransac",
    "fit_line_lsq",
    "compute_path_errors_arc",
    "compute_path_errors_line",
]
