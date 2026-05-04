"""光度标定独立脚本（plan §5.3 + §11.1 tools/）。

**用途**：在 K230 上跑一次，把当前赛场光照下的 ROI Otsu / μ_bg / σ_bg /
line_threshold 采集 30 帧后落盘到 ``config.PHOTO_CALIB_PATH``。

**与运行期 `Photometric.bootstrap()` 的关系**：算法等价（共享同一份累积逻辑），
区别只在：

- 本脚本只跑一次、直接退出，不进主循环；
- 把结果写到独立 JSON（``calib_photometric.json``），不污染阶段 C 的
  ``calib.json``；
- 控制台打印更详尽，方便观察光照场景的真实 μ_bg / σ_bg 分布。

**用法**：

    在 CanMV IDE 打开本文件运行；运行结束后控制台会打印：

        [photometric] bootstrap done frames=30 mu_bg=... sigma_bg=...
                      thr_otsu=... threshold=...
        [calibrate_photometric] saved to /sdcard/calib_photometric.json

    主入口 ``vision_line_tracking.py`` 不强制读取这份 JSON；阶段 C 接 IPM
    时再统一并入 ``calib.json``。
"""

import os
import sys
import time

try:
    import ujson as json
except ImportError:
    import json

# 让 tools/ 下的脚本能直接 import 顶层 vision/
# CanMV IDE 默认 cwd 在脚本目录；显式追加上一级路径即可。
_HERE = sys.path[0] if sys.path else ""
if _HERE.endswith("/tools") or _HERE.endswith("\\tools"):
    sys.path.insert(0, _HERE.rsplit("tools", 1)[0].rstrip("/\\"))

import config
from vision.camera import Camera
from vision.photometric import Photometric


def _ensure_dir_for(path):
    """光度标定 JSON 通常落到 /sdcard/ 根，如果路径含子目录则确保它存在。"""
    idx = path.rfind("/")
    if idx <= 0:
        return True
    parent = path[:idx]
    try:
        os.stat(parent)
        return True
    except OSError:
        try:
            os.mkdir(parent)
            return True
        except OSError as e:
            print("[calibrate_photometric] mkdir failed:", parent, e)
            return False


def _save_json(path, payload):
    if not _ensure_dir_for(path):
        return False
    try:
        with open(path, "w") as f:
            f.write(json.dumps(payload))
        return True
    except Exception as e:
        print("[calibrate_photometric] write JSON failed:", path, e)
        return False


def main():
    print(
        "[calibrate_photometric] start  config=%s  frames=%d  roi=%s"
        % (
            config.CONFIG_VERSION,
            config.PHOTO_BOOTSTRAP_FRAMES,
            str(config.ROI_TOTAL_PX),
        )
    )

    camera = Camera()
    camera.init()
    camera.log_configured_modes()
    camera.start()

    photo = Photometric()

    try:
        # 让 sensor 进入稳态曝光，丢弃前几帧。
        for _ in range(5):
            camera.read_algo_frame()

        ok = photo.calibrate_blocking(camera)
        if not ok:
            print("[calibrate_photometric] FAILED to collect frames")
            return

        payload = {
            "config_version": config.CONFIG_VERSION,
            "ts_ms": time.ticks_ms(),
            "roi_total_px": list(config.ROI_TOTAL_PX),
            "n_frames": config.PHOTO_BOOTSTRAP_FRAMES,
            "fallback_k_sigma": config.PHOTO_FALLBACK_K_SIGMA,
            "mu_bg": round(photo.mu_bg, 2),
            "sigma_bg": round(photo.sigma_bg, 2),
            "thr_otsu": photo.thr_otsu,
            "line_threshold": photo.threshold,
        }

        path = config.PHOTO_CALIB_PATH
        if _save_json(path, payload):
            print("[calibrate_photometric] saved to %s" % path)
            print("[calibrate_photometric] payload:", payload)
        else:
            print("[calibrate_photometric] not saved; result still printed above")

    except KeyboardInterrupt as e:
        print("[calibrate_photometric] user stop:", e)
    except BaseException as e:
        print("[calibrate_photometric] exception:", e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        camera.stop()
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass


if __name__ == "__main__":
    main()
