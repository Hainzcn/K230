"""IPM 四点单应标定（plan §5.2 + §11.1 tools/）。

**纯 PC 端脚本**（CPython + numpy）。在 PC 上运行::

    cd /path/to/K230
    python tools/calibrate_ipm.py

它会：

1. 读取脚本顶部 ``CORRESPONDENCES`` 里的 4 对像素 ↔ 地面坐标对应关系；
2. 用 DLT（Direct Linear Transform）求解 3×3 单应 ``H``（像素 (u,v,1) →
   地面 (x_g,y_g,1)）；
3. 校验 4 个标靶点的来回误差，期望 < 1 px / < 1 mm；
4. 写 ``/sdcard/calib.json`` 的 ``ipm`` 节（schema 与 ``config.py`` 顶部
   注释一致），把生成的 JSON 放在 ``CALIB_OUT_PATH``。

之后用户把生成的 ``calib.json`` **手工拷到 K230 SD 卡根目录**，K230 端
``vision_line_tracking.py`` 启动期 ``config.load_calibration()`` 会自动
读取并喂给 ``vision.ground_mapper.GroundMapper``。

本脚本不接 K230 sensor / cv_lite / image，是为了在没插 K230 的工位上也能
（基于 PC 上选点的 4 对对应）解出 H；K230 端的"屏幕辅助打靶"工具登记在
``docs/task_log/phase_C.md`` 的 TODO，不在阶段 C 主交付里。

如何选 4 个标靶
================

1. 在赛道上摆 4 个十字小靶（建议构成一个 300 mm × 200 mm 的梯形或矩形），
   各自在车体地面坐标系下用尺子量出 ``(x_g, y_g)`` mm（``x_g`` 向前为正、
   ``y_g`` 向左为正；plan §2.3）。
2. 让 K230 摄像头在 **算法分辨率 320×240** 上拍一张样张
   （``vision_line_tracking.py`` 里 ``CAPTURE_ENABLE=True`` 即可），用
   PC 上任意支持像素读取的图像查看器（GIMP / Photoshop / IrfanView /
   matplotlib imshow + click event）量出每个十字中心在该样张上的 ``(u, v)``
   像素坐标（注意：必须是 **算法分辨率 320×240**，不是显示分辨率）。
3. 把 4 对填到下面 ``CORRESPONDENCES``，按"右下、右上、左上、左下"顺序
   填好（顺序可任意，但保持记录便于回溯）。
4. 跑本脚本，确认控制台打印的 4 个点的来回误差 < 1px / < 1mm。

**最小验证点**：可以先用脚本顶部预填的 ``CORRESPONDENCES`` 跑一次，得到
"占位 H" 用于桌面 bench 测试；这条数据不是真实标定值，仅作功能验证。
"""

import json
import os
import sys
import time

try:
    import numpy as np
except ImportError:
    print("[calibrate_ipm] FATAL: numpy is required on PC. Run `pip install numpy`.")
    sys.exit(1)

# ---------------------------------------------------------------------------
# **用户填写区**：把 4 对 (像素, 地面 mm) 对应填到这里。
# 像素必须在算法分辨率（320 × 240）下；地面坐标用 mm。
# ---------------------------------------------------------------------------
ALGO_W = 320
ALGO_H = 240

# 占位示例：基于 plan §4.1（h_cam=120 mm, θ_pitch=20°）的解析推导来"凑"
# 出来的合理 4 对，仅供脚本流程验证；真实标定时整组替换为实测值。
# 解析推导的几何意义：
#   - NEAR 中心列 (u=160, v=222) ≈ 地面 (x_g=170 mm, y_g=0)
#   - FAR  中心列 (u=160, v=80)  ≈ 地面 (x_g=720 mm, y_g=0)
#   - 左缘 / 右缘按 mm/px 估值取 ±100 mm 横向。
CORRESPONDENCES = [
    # (u_px, v_px, x_g_mm, y_g_mm)
    (210.0, 222.0,  170.0,  -110.0),   # 右下：近距右
    (203.0,  82.0,  720.0,  -260.0),   # 右上：远距右
    (117.0,  82.0,  720.0,   260.0),   # 左上：远距左
    (110.0, 222.0,  170.0,   110.0),   # 左下：近距左
]

# 输出 JSON 路径。脚本执行目录下的相对路径；用户拷到 K230 SD 卡根。
CALIB_OUT_PATH = os.path.join(os.path.dirname(__file__), "..", "calib.json")
CONFIG_VERSION = "phaseC-0.1"


def solve_homography_dlt(corrs):
    """4 对 (u, v) ↔ (X, Y) 解 3×3 单应矩阵 H，row-major 9-tuple 返回。

    DLT：每对生成 2 行 8x9 系数矩阵 A，最后一行取 ||h|| = 1 约束（SVD
    取最小奇异向量）。即使 corrs > 4，也走 SVD 解超定 LSQ 系统。

    :param corrs: list of (u, v, X, Y) tuples，至少 4 对。
    :return: 9 元素 list，对应 (h11, h12, h13, h21, h22, h23, h31, h32, h33)；
             失败返回 None。
    """
    if len(corrs) < 4:
        print("[calibrate_ipm] need ≥ 4 correspondences, got %d" % len(corrs))
        return None
    A = []
    for (u, v, X, Y) in corrs:
        A.append([-u, -v, -1.0,  0.0,  0.0,  0.0,  X * u,  X * v,  X])
        A.append([0.0,  0.0,  0.0, -u,  -v,  -1.0,  Y * u,  Y * v,  Y])
    A = np.asarray(A, dtype=np.float64)
    # SVD：H 是 A 最小奇异值对应的右奇异向量。
    _, _, vt = np.linalg.svd(A)
    h = vt[-1]
    if abs(h[8]) > 1e-12:
        h = h / h[8]    # 习惯归一化使 h33 = 1（不强制，但便于阅读）
    return [float(x) for x in h]


def apply_H(H, u, v):
    """把 H @ (u, v, 1) 齐次归一化为 (X, Y)。"""
    h = H
    w = h[6] * u + h[7] * v + h[8]
    X = (h[0] * u + h[1] * v + h[2]) / w
    Y = (h[3] * u + h[4] * v + h[5]) / w
    return X, Y


def invert_H(H):
    """3x3 矩阵手工求逆（DLT 的 reverse 校验用）。"""
    M = np.asarray(H, dtype=np.float64).reshape(3, 3)
    return [float(x) for x in np.linalg.inv(M).reshape(-1)]


def verify_correspondences(H, corrs):
    """打印 4 对来回误差。返回 (max_err_forward_mm, max_err_back_px)。"""
    max_fwd = 0.0
    max_bwd = 0.0
    H_inv = invert_H(H)
    print("[calibrate_ipm] verify forward (pixel -> ground):")
    print("  idx   u     v       X_actual  Y_actual    X_pred    Y_pred    err_mm")
    for i, (u, v, X, Y) in enumerate(corrs):
        Xp, Yp = apply_H(H, u, v)
        err = ((Xp - X) ** 2 + (Yp - Y) ** 2) ** 0.5
        if err > max_fwd:
            max_fwd = err
        print("  %d  %5.1f %5.1f   %8.2f %8.2f   %8.2f %8.2f   %7.3f"
              % (i, u, v, X, Y, Xp, Yp, err))
    print("[calibrate_ipm] verify reverse (ground -> pixel):")
    print("  idx   X     Y       u_actual  v_actual    u_pred    v_pred    err_px")
    for i, (u, v, X, Y) in enumerate(corrs):
        up, vp = apply_H(H_inv, X, Y)
        err = ((up - u) ** 2 + (vp - v) ** 2) ** 0.5
        if err > max_bwd:
            max_bwd = err
        print("  %d  %6.1f %6.1f   %8.2f %8.2f   %8.2f %8.2f   %7.3f"
              % (i, X, Y, u, v, up, vp, err))
    return max_fwd, max_bwd


def write_calib_json(path, H, corrs):
    payload = {
        "config_version": CONFIG_VERSION,
        "ts_ms": int(time.time() * 1000),
        "ipm": {
            "H_3x3": [float(x) for x in H],
            "image_wh": [ALGO_W, ALGO_H],
            "corners_image": [[float(c[0]), float(c[1])] for c in corrs],
            "corners_ground_mm": [[float(c[2]), float(c[3])] for c in corrs],
        },
    }
    abs_path = os.path.abspath(path)
    parent = os.path.dirname(abs_path)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(abs_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    return abs_path


def main():
    print("[calibrate_ipm] config_version=%s  algo=%dx%d  pairs=%d"
          % (CONFIG_VERSION, ALGO_W, ALGO_H, len(CORRESPONDENCES)))

    H = solve_homography_dlt(CORRESPONDENCES)
    if H is None:
        print("[calibrate_ipm] FAILED to solve H")
        return 1

    print("[calibrate_ipm] H (row-major):")
    for row in range(3):
        print("  [%12.4f %12.4f %12.4f]"
              % (H[3 * row + 0], H[3 * row + 1], H[3 * row + 2]))

    fwd, bwd = verify_correspondences(H, CORRESPONDENCES)
    print("[calibrate_ipm] max forward err = %.3f mm  max reverse err = %.3f px"
          % (fwd, bwd))
    if fwd > 5.0 or bwd > 1.0:
        print("[calibrate_ipm] WARNING: residuals exceed expected (≤1 px / ≤1 mm).")
        print("[calibrate_ipm] Check correspondences for typos / wrong order.")

    out_path = write_calib_json(CALIB_OUT_PATH, H, CORRESPONDENCES)
    print("[calibrate_ipm] wrote %s" % out_path)
    print("[calibrate_ipm] copy this file to K230 SD card root as /sdcard/calib.json")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
