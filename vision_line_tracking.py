"""K230 视觉循迹主入口（阶段 A：摄像头基础采集）。

阶段 A 不做任何检测/控制，仅完成：

1. CHN0 / CHN1 双通道初始化与显示绑定（``vision.camera.Camera``）。
2. OSD 叠加：``FPS <algo_fps> (T <period_ms>ms)`` 一行，1 位小数。
   阶段 A 主循环几乎不做事，``snapshot()`` 是阻塞拉帧，因此 algo_fps
   同时反映 sensor 对 CHN1 的实际供帧率——单一指标就够，不再引入
   ``Display.fps()``（那是 LCD VSync，不是 stream 速率，前两轮错把它当
   分母已修正）。``period_ms`` 超过 ``FRAME_PERIOD_ALERT_MS`` 时整行标红。
   剩余内存仅在漂移 ≥ ``MEM_DRIFT_ALERT_PCT`` 或低于 ``MEM_LOW_ALERT_BYTES``
   时才追加显示（同样标红），平时保持 OSD 清爽。
3. ROI 框的等比例叠加（便于物理装配阶段对镜头视野）。
4. ``KeyboardInterrupt`` / 异常 / 资源清理的统一保护。
5. （可选）按 ``config.CAPTURE_*`` 配置周期性把 CHN1 灰度帧落盘为 JPEG，
   用于满足 plan §12 阶段 A 的 "拍摄静态赛道样张 ≥ 100 张" 任务。

后续阶段（B 起）将在主循环里替换 / 追加：
``photometric → line_detector → ground_mapper → geometry → estimator
→ controller → comms``，但本文件仍只负责装配，不写算法细节
（plan §11.3）。
"""

import gc
import os
import sys
import time

import config
from vision.camera import Camera


def _ensure_dir(path):
    """SD 卡上确保目录存在；不存在就创建。"""
    try:
        os.stat(path)
        return True
    except OSError:
        try:
            os.mkdir(path)
            return True
        except OSError as e:
            print("[VLT] mkdir failed:", path, e)
            return False


def _setup_capture():
    """根据 config 决定是否开启采样模式，返回布尔 enable。"""
    if not config.CAPTURE_ENABLE:
        return False
    ok = _ensure_dir(config.CAPTURE_DIR)
    if ok:
        print(
            "[VLT] capture mode ON: dir=%s, every=%d frames, max=%d"
            % (
                config.CAPTURE_DIR,
                config.CAPTURE_INTERVAL_FRAMES,
                config.CAPTURE_MAX_SAMPLES,
            )
        )
    return ok


def main():
    print(
        "[VLT] vision_line_tracking start, config=%s, debug=%s"
        % (config.CONFIG_VERSION, config.DEBUG_DISPLAY)
    )

    if config.GC_THRESHOLD_BYTES > 0:
        try:
            gc.threshold(config.GC_THRESHOLD_BYTES)
        except Exception as e:
            print("[VLT] gc.threshold not supported:", e)

    capture_enabled = _setup_capture()
    capture_count = 0

    camera = Camera()
    camera.init()
    # 诊断：打印本次请求的 sensor mode 与各通道实际输出尺寸。与驱动日志里
    # ``find sensor ..., output WxH@FPS`` 一行对照，就能看出请求是否被接受。
    camera.log_configured_modes()
    camera.start()

    # 启动期 raw snapshot 计时探针：用于把 snapshot 自身耗时和主循环其他
    # 开销分开。设 PROBE_SNAPSHOT_FRAMES=0 关闭。
    if config.PROBE_SNAPSHOT_FRAMES > 0:
        camera.probe_snapshot_timing(config.PROBE_SNAPSHOT_FRAMES)

    # 内存监测：plan §12 阶段 A 验收要求 mem_free 震荡 ≤ 10%
    mem0 = gc.mem_free()
    mem_min = mem0
    mem_max = mem0
    last_mem = mem0

    last_log_ms = time.ticks_ms()
    snapshot_fail_streak = 0

    try:
        while True:
            os.exitpoint()

            img = camera.read_algo_frame()
            if img is None:
                snapshot_fail_streak += 1
                if snapshot_fail_streak >= 10:
                    print("[VLT] snapshot failed 10 times in a row")
                    snapshot_fail_streak = 0
                continue
            snapshot_fail_streak = 0

            # 采样落盘
            if (
                capture_enabled
                and capture_count < config.CAPTURE_MAX_SAMPLES
                and (camera.frame_count() % config.CAPTURE_INTERVAL_FRAMES == 0)
            ):
                fname = "%s/frame_%05d.jpg" % (config.CAPTURE_DIR, capture_count)
                if camera.save_algo_frame(img, fname):
                    capture_count += 1
                    if capture_count % 10 == 0:
                        print(
                            "[VLT] captured %d / %d frames"
                            % (capture_count, config.CAPTURE_MAX_SAMPLES)
                        )
                if capture_count == config.CAPTURE_MAX_SAMPLES:
                    print(
                        "[VLT] capture quota reached (%d), stop saving"
                        % capture_count
                    )

            now = time.ticks_ms()
            if camera.maybe_update_fps(now):
                cur_mem = gc.mem_free()
                if cur_mem < mem_min:
                    mem_min = cur_mem
                if cur_mem > mem_max:
                    mem_max = cur_mem
                last_mem = cur_mem

                mem_drift_pct = (
                    100.0 * (mem_max - mem_min) / mem_max if mem_max > 0 else 0.0
                )

                algo_fps = camera.algo_fps()
                period_ms = camera.algo_period_ms()

                # FPS 行：algo_fps + 帧周期，均 1 位小数；
                # period > 阈值时整行标红（性能降级第一指标）。
                fps_text = "FPS %5.1f  (T %5.1f ms)" % (algo_fps, period_ms)
                fps_color = (
                    config.OSD_ALERT_COLOR
                    if period_ms > config.FRAME_PERIOD_ALERT_MS
                    else None
                )
                lines = [(fps_text, fps_color)]

                mem_alert = (
                    mem_drift_pct >= config.MEM_DRIFT_ALERT_PCT
                    or cur_mem < config.MEM_LOW_ALERT_BYTES
                )
                if mem_alert:
                    lines.append((
                        "MEM %d KB  (drift %.1f%%)"
                        % (cur_mem // 1024, mem_drift_pct),
                        config.OSD_ALERT_COLOR,
                    ))

                if capture_enabled:
                    lines.append(
                        "CAP %d/%d"
                        % (capture_count, config.CAPTURE_MAX_SAMPLES)
                    )
                camera.render_overlay(lines)

            # 控制台日志节流：完整指标依然落日志，便于离线分析（plan §13.1）。
            if time.ticks_diff(now, last_log_ms) >= config.LOG_INTERVAL_MS:
                last_log_ms = now
                mem_drift_pct = (
                    100.0 * (mem_max - mem_min) / mem_max if mem_max > 0 else 0.0
                )
                print(
                    "[VLT] algo_fps=%.1f period=%.1fms frames=%d "
                    "mem=%d (min=%d max=%d drift=%.1f%%)"
                    % (
                        camera.algo_fps(),
                        camera.algo_period_ms(),
                        camera.frame_count(),
                        last_mem,
                        mem_min,
                        mem_max,
                        mem_drift_pct,
                    )
                )

    except KeyboardInterrupt as e:
        print("[VLT] user stop:", e)
    except BaseException as e:
        print("[VLT] exception: %s" % e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        camera.stop()
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception as e:
            print("[VLT] exitpoint failed:", e)
        time.sleep_ms(100)
        try:
            gc.collect()
        except Exception:
            pass
        print(
            "[VLT] stopped, frames=%d, last_mem=%d, mem_range=%d~%d"
            % (
                camera.frame_count() if camera else -1,
                gc.mem_free(),
                mem_min,
                mem_max,
            )
        )


if __name__ == "__main__":
    main()
