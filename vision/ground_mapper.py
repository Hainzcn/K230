"""IPM 地面映射（plan §5.2）。

把阶段 B 的扫描带像素质心 ``bands[i].cx_px`` 映射到地面坐标系（mm），
为阶段 C 的 RANSAC 圆弧拟合 / 一阶回归 fallback / 控制律提供输入。

设计要点
=========

- **不构建 ROI 全像素 LUT**：detector 每帧只产出 5 个候选点（5 条扫描带的
  cx），全像素 LUT 是浪费。直接做 9 元素 H 矩阵展开（无 ulab 依赖、躲掉
  阶段 B task_log §4 的 ulab quirk）。每帧 5 × (9 mul + 6 add + 2 div)
  ≈ 0.05 ms，比 plan §5.2 给的两种实现都更省。

- **三档加载状态**（``mode``）：

  - ``"calibrated"``：``calib.json`` 里有效 ``ipm.H_3x3``，9 个浮点 + ALGO
    分辨率匹配通过；这是装车后的稳态。
  - ``"default"``：calib 缺失但 plan §4.1 安装几何（``MOUNT_H_CAM_MM``、
    ``MOUNT_PITCH_DEG``）能解析推导一份占位 H；OSD 应显式标
    ``CALIB:DEFAULT`` 提醒"e_y / ψ_e 数值有几十 mm 系统偏差"。桌面 bench
    主要走这条路径（不装车也能验证全链路）。
  - ``"none"``：连解析推导都失败（极少见，只在配置参数都被改坏时触发）；
    OSD 显示 ``NO CALIB``，主循环跳过 IPM/RANSAC/EMA 链路。

- **坐标系约定**（plan §1.3 / §2.3）：

  - 输入像素 ``(u, v)``：算法分辨率（``ALGO_WIDTH × ALGO_HEIGHT``）下的
    图像坐标，原点在左上角，u→右、v→下。
  - 输出地面 ``(x_g_mm, y_g_mm)``：车体 / 地面坐标系（B 与 G 重合），
    原点在两轮中点地面投影下方，``x_g`` 向前为正、``y_g`` 向左为正。
  - 与 plan §1.3 ``e_y`` "正号偏右" 的符号映射放到 ``geometry``
    ``compute_path_errors_*`` 里做（GroundMapper 只输出原始几何量，不
    决定符号）。

- **解析占位 H 推导**（``_derive_fallback_H``）：

  针孔模型 + 平面假设。镜头中心在 ``(0, 0, h_cam)``，光轴沿世界 +x 方向
  下倾 ``θ_pitch``（plan §4.1）。代数推导得到从像素 ``(u, v, 1)`` 到地面
  ``(x_g, y_g, 1)`` 的单应矩阵 ``H``；详见 ``_derive_fallback_H`` 里的
  注释。``yaw`` 暂未注入（默认 0）：plan §4.1 要求实测偏差 ≤ 1°，且
  yaw ≠ 0 时只是给 H 多乘一个绕 z 轴的旋转矩阵，待装车后实测如果需要再
  补；不影响 fallback 主路径。
"""

import math

import config


class GroundPoint:
    """单个像素 → 地面坐标的映射结果。"""

    __slots__ = ("band_idx", "u_px", "v_px", "x_g_mm", "y_g_mm", "valid")

    def __init__(self, band_idx=-1):
        self.band_idx = band_idx
        self.u_px = -1.0
        self.v_px = -1.0
        self.x_g_mm = 0.0
        self.y_g_mm = 0.0
        self.valid = False


def _invert_3x3(m):
    """返回 3x3 矩阵的逆（输入输出都是 9 元素 tuple）。

    无 ulab 依赖：阶段 B task_log §4 记录过 ulab ``np.dot`` / ``axis sum``
    / 加权质心广播等多个 quirk。这里走纯标量算术，可移植到 PC 单测。

    奇异矩阵返回 ``None``，由调用方决定降级。
    """
    a, b, c, d, e, f, g, h, i = m
    # 余子式
    A = e * i - f * h
    B = -(d * i - f * g)
    C = d * h - e * g
    D = -(b * i - c * h)
    E = a * i - c * g
    F = -(a * h - b * g)
    G = b * f - c * e
    H = -(a * f - c * d)
    I = a * e - b * d
    det = a * A + b * B + c * C
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return (
        A * inv_det, D * inv_det, G * inv_det,
        B * inv_det, E * inv_det, H * inv_det,
        C * inv_det, F * inv_det, I * inv_det,
    )


def _apply_3x3(m, x, y):
    """``m @ [x, y, 1]^T`` 并齐次归一。返回 ``(X, Y)`` 或 ``None``（齐次分量 ≈ 0）。"""
    a, b, c, d, e, f, g, h, i = m
    w = g * x + h * y + i
    if abs(w) < 1e-9:
        return None
    inv_w = 1.0 / w
    X = (a * x + b * y + c) * inv_w
    Y = (d * x + e * y + f) * inv_w
    return X, Y


def _derive_fallback_H(cfg):
    """基于 plan §4.1 安装几何反推占位 H。返回 9 元素 tuple 或 ``None``。

    几何推导（针孔模型 + 平面假设，镜头中心在 ``(0, 0, h_cam)``）::

        u' = (u - cx) / fx
        v' = (v - cy) / fy
        D  = sin θ + v' · cos θ            # 必须 > 0（否则像素朝水平线以上看，无地面解）
        x_g =  h_cam · (cos θ - v' · sin θ) / D
        y_g = -h_cam · u' / D

    令 ``w = fy · D = fy sin θ + (v − cy) cos θ`` 为齐次分量，则单应::

        H = | 0                 -h sin θ                h(fy cos θ + cy sin θ) |
            | -h fy / fx           0                       h fy cx / fx        |
            | 0                  cos θ                  fy sin θ - cy cos θ    |

    其中 ``h = h_cam``、``θ = θ_pitch``。``H @ (u,v,1)^T = (x_g · w, y_g · w, w)``，
    齐次归一即得地面坐标。

    fx / fy 由 plan §4.1 给的 SENSOR_HFOV_DEG / SENSOR_VFOV_DEG + 算法分辨率
    （``ALGO_WIDTH × ALGO_HEIGHT``）反推；cx / cy 取算法分辨率正中心。

    yaw / 镜头畸变 / 装配垂直度都未注入——这只是 fallback，装车后由
    ``tools/calibrate_ipm.py`` 解出 H 覆盖。
    """
    h_cam = float(getattr(cfg, "MOUNT_H_CAM_MM", 0.0))
    pitch_deg = float(getattr(cfg, "MOUNT_PITCH_DEG", 0.0))
    if h_cam <= 0.0:
        return None
    if not (1.0 <= pitch_deg <= 60.0):
        return None  # 角度退化，光轴几乎水平 / 朝下，IPM 不成立

    theta = pitch_deg * math.pi / 180.0
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    if sin_t < 1e-6:
        return None

    hfov = float(getattr(cfg, "SENSOR_HFOV_DEG", 54.0)) * math.pi / 180.0
    vfov = float(getattr(cfg, "SENSOR_VFOV_DEG", 41.0)) * math.pi / 180.0
    W = float(cfg.ALGO_WIDTH)
    H_img = float(cfg.ALGO_HEIGHT)
    fx = (W * 0.5) / math.tan(hfov * 0.5)
    fy = (H_img * 0.5) / math.tan(vfov * 0.5)
    cx = W * 0.5
    cy = H_img * 0.5

    h11 = 0.0
    h12 = -h_cam * sin_t
    h13 = h_cam * (fy * cos_t + cy * sin_t)
    h21 = -h_cam * fy / fx
    h22 = 0.0
    h23 = h_cam * fy * cx / fx
    h31 = 0.0
    h32 = cos_t
    h33 = fy * sin_t - cy * cos_t
    return (h11, h12, h13, h21, h22, h23, h31, h32, h33)


class GroundMapper:
    """把图像像素映射到地面 (x_g, y_g) mm 的单应映射器。

    使用流程::

        mapper = GroundMapper()
        mapper.load(config.load_calibration())
        if mapper.is_calibrated():
            gp_list = mapper.bands_to_ground(detection.bands)
            ...

    ``mode`` ∈ {``"calibrated"``, ``"default"``, ``"none"``}；前两档
    ``is_calibrated()`` 都返回 True，但 ``mode == "default"`` 时 OSD 应
    标 ``CALIB:DEFAULT`` 提醒用户系统偏差较大。
    """

    __slots__ = (
        "cfg",
        "_H",
        "_H_inv",
        "_mode",
        "_load_error",
        "_buf",
        "_buf_size",
    )

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else config
        self._H = None       # 9-tuple (row-major)
        self._H_inv = None   # 9-tuple
        self._mode = "none"
        self._load_error = ""
        # 预分配 GroundPoint 缓冲，避免帧循环 alloc（plan §9.2 守则 5）。
        self._buf_size = int(getattr(self.cfg, "BAND_COUNT", 5))
        self._buf = [GroundPoint(i) for i in range(self._buf_size)]

    # ---- 加载 ----

    def load(self, calib_dict=None):
        """从 calib dict 加载 H；缺失则尝试解析占位 H；都失败则 mode="none"。

        :param calib_dict: ``config.load_calibration()`` 返回值。``None``
            视为空 dict（直接走 fallback）。
        :return: ``self.mode`` 字符串。
        """
        self._H = None
        self._H_inv = None
        self._mode = "none"
        self._load_error = ""

        if calib_dict is None:
            calib_dict = {}

        ipm = calib_dict.get("ipm") if isinstance(calib_dict, dict) else None
        if isinstance(ipm, dict):
            h = ipm.get("H_3x3")
            if isinstance(h, (list, tuple)) and len(h) == 9:
                ok = True
                vals = []
                for v in h:
                    try:
                        vals.append(float(v))
                    except Exception:
                        ok = False
                        break
                if ok:
                    self._set_H(tuple(vals), "calibrated")
                    if self._H_inv is None:
                        self._mode = "none"
                        self._H = None
                        self._load_error = "H singular (calibrated)"
                    else:
                        return self._mode
                else:
                    self._load_error = "H_3x3 contains non-numeric"

        # fallback
        fb = _derive_fallback_H(self.cfg)
        if fb is not None:
            self._set_H(fb, "default")
            if self._H_inv is None:
                self._mode = "none"
                self._H = None
                self._load_error = "fallback H singular"
            else:
                return self._mode

        self._load_error = self._load_error or "no calibration and no fallback"
        return self._mode

    def _set_H(self, H, mode):
        self._H = H
        self._H_inv = _invert_3x3(H)
        self._mode = mode

    # ---- 状态 ----

    @property
    def mode(self):
        return self._mode

    def is_calibrated(self):
        return self._mode in ("calibrated", "default")

    def is_default(self):
        return self._mode == "default"

    def load_error(self):
        return self._load_error

    def H_matrix(self):
        """返回 9 元素 tuple；未加载时为 None。仅诊断 / 单测用。"""
        return self._H

    # ---- 映射 ----

    def pixel_to_ground(self, u, v):
        """单像素 → 地面坐标。返回 ``(x_g_mm, y_g_mm)`` 或 ``None``。"""
        if self._H is None:
            return None
        return _apply_3x3(self._H, float(u), float(v))

    def ground_to_pixel(self, x_mm, y_mm):
        """地面坐标 → 像素。返回 ``(u, v)`` 或 ``None``。

        OSD 用：把圆心 / 切线锚点反投回算法分辨率坐标。
        """
        if self._H_inv is None:
            return None
        return _apply_3x3(self._H_inv, float(x_mm), float(y_mm))

    def bands_to_ground(self, bands):
        """把 5 条扫描带的 cx 像素映射到地面坐标。

        每条带取 ``(u, v) = (cx_px, (y_top + y_bot) / 2)``——cx 在带的几何
        中心 y 处生效。无效带（``valid=False``）跳过映射但占位返回。

        :param bands: ``DetectionResult.bands`` 列表
        :return: list[GroundPoint]，长度 = ``len(bands)``，与 bands 一一
            对应。``GroundPoint.valid=True`` 表示该带 + 映射都成功。
        """
        n = len(bands)
        if n > self._buf_size:
            self._buf = [GroundPoint(i) for i in range(n)]
            self._buf_size = n
        for i in range(n):
            gp = self._buf[i]
            gp.band_idx = bands[i].idx
            gp.valid = False
            gp.u_px = -1.0
            gp.v_px = -1.0
            gp.x_g_mm = 0.0
            gp.y_g_mm = 0.0
            if not bands[i].valid:
                continue
            if self._H is None:
                continue
            u = float(bands[i].cx_px)
            v = 0.5 * (float(bands[i].y_top) + float(bands[i].y_bot))
            mapped = _apply_3x3(self._H, u, v)
            if mapped is None:
                continue
            gp.u_px = u
            gp.v_px = v
            gp.x_g_mm = mapped[0]
            gp.y_g_mm = mapped[1]
            gp.valid = True
        return self._buf[:n]

    def self_test(self):
        """启动期自检：对几个图像中心点做映射，确保 H 不奇异。"""
        if self._H is None:
            return False
        u_c = self.cfg.ALGO_WIDTH * 0.5
        v_c = self.cfg.ALGO_HEIGHT * 0.5
        m = self.pixel_to_ground(u_c, v_c)
        if m is None:
            return False
        # 反向校验
        if self._H_inv is None:
            return False
        back = self.ground_to_pixel(m[0], m[1])
        if back is None:
            return False
        du = back[0] - u_c
        dv = back[1] - v_c
        return (du * du + dv * dv) < 1e-2  # 来回误差 < 0.1 px


__all__ = ["GroundMapper", "GroundPoint"]
