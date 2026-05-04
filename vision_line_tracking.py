"""K230 视觉循迹主入口（阶段 A 摄像头采集 + 阶段 B 多扫描带检测）。

本文件**只负责装配**，不写算法细节（plan §11.3）。

阶段 A（继承）：
- CHN0 / CHN1 双通道初始化与显示绑定（``vision.camera.Camera``）；
- OSD 叠加：``FPS / T / Q_L2 / valid / cx_near`` 文本行 + ROI 框 + 检测可视化；
- ``KeyboardInterrupt`` / 异常 / 资源清理的统一保护；
- 可选采样模式（``config.CAPTURE_*``）。

阶段 B（新增）：
- ``vision.photometric.Photometric`` 启动期跑 30 帧 bootstrap 拿到 line_threshold；
  运行期每 ~1 s 检查 ``μ_bg`` 漂移，超阈值就在背景任务里渐进重标定（不阻塞）。
- ``vision.line_detector.LineDetector`` 每帧跑 L0 + (可选 L1) + L2，输出
  5 条扫描带的 cx / mass / width / valid 与 Q_L2 评分。
- OSD 增加：5 条扫描带边框、每带 cx 圆点（valid 绿/invalid 红）、宽度水平线段、
  Q_L2 数值文本行；控制台 5 s 节流日志增加 ``q_l2 / valid / cx_near``。

后续阶段（C 起）将在主循环里追加：
``ground_mapper → geometry → estimator → controller → comms``，
仍维持本文件 "装配 only" 的职责边界。
"""

import gc
import os
import sys
import time

import config
from vision.camera import Camera
from vision.photometric import Photometric
from vision.line_detector import LineDetector
from vision.quality import grade as q_grade


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

    # 阶段 B：光度 bootstrap + 检测器装配（plan §5.3 + §6.1）
    photometric = Photometric()
    detector = LineDetector()
    if not detector.self_test():
        print("[VLT] line_detector self_test FAILED; abort")
        camera.stop()
        return
    photometric.bootstrap(camera)
    if not photometric.self_test():
        print("[VLT] photometric self_test FAILED; using LINE_THRESHOLD_INIT fallback")

    # 内存监测：plan §12 阶段 A 验收要求 mem_free 震荡 ≤ 10%
    mem0 = gc.mem_free()
    mem_min = mem0
    mem_max = mem0
    last_mem = mem0

    last_log_ms = time.ticks_ms()
    snapshot_fail_streak = 0
    detection = None
    img = None

    try:
        while True:
            os.exitpoint()
            # ticks_ms 放在循环顶部，即使 snapshot 持续失败也能让日志块按
            # LOG_INTERVAL_MS 节流地报警，不至于静默卡死。
            now = time.ticks_ms()

            img = camera.read_algo_frame()
            if img is None:
                snapshot_fail_streak += 1
                if snapshot_fail_streak >= 10:
                    print("[VLT] snapshot failed 10 times in a row")
                    snapshot_fail_streak = 0
                continue
            snapshot_fail_streak = 0

            # 阶段 B 主算法：光度自适应 + 多扫描带检测
            photometric.update(img, now)
            detection = detector.process(img, photometric.threshold)

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

                # 阶段 B：Q_L2 + 有效带数 + cx_near + 阈值
                q_color = None
                if detection.q_l2 < config.Q_HOLD:
                    q_color = config.OSD_ALERT_COLOR
                lines.append((
                    "Q %5.1f  V %d/%d  cxN %s  thr %d"
                    % (
                        detection.q_l2,
                        detection.n_valid,
                        config.BAND_COUNT,
                        ("%5.1f" % detection.cx_near_px)
                        if detection.cx_near_px >= 0
                        else "  -- ",
                        detection.threshold_used,
                    ),
                    q_color,
                ))

                if photometric.is_recalibrating:
                    lines.append((
                        "PHOTO recal %d/%d"
                        % (
                            photometric.recalib_counter,
                            config.PHOTO_BOOTSTRAP_FRAMES,
                        ),
                        config.OSD_ALERT_COLOR,
                    ))

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
                camera.render_overlay(lines, detection=detection)

            # 控制台日志节流：完整指标依然落日志，便于离线分析（plan §13.1）。
            if (
                detection is not None
                and time.ticks_diff(now, last_log_ms) >= config.LOG_INTERVAL_MS
            ):
                last_log_ms = now
                mem_drift_pct = (
                    100.0 * (mem_max - mem_min) / mem_max if mem_max > 0 else 0.0
                )
                print(
                    "[VLT] algo_fps=%.1f period=%.1fms frames=%d "
                    "Q=%.1f(%s) V=%d/%d cxN=%.1f cxF=%.1f thr=%d "
                    "mu=%.1f sig=%.1f%s "
                    "mem=%d (min=%d max=%d drift=%.1f%%)"
                    % (
                        camera.algo_fps(),
                        camera.algo_period_ms(),
                        camera.frame_count(),
                        detection.q_l2,
                        q_grade(detection.q_l2),
                        detection.n_valid,
                        config.BAND_COUNT,
                        detection.cx_near_px,
                        detection.cx_far_px,
                        detection.threshold_used,
                        photometric.mu_bg,
                        photometric.sigma_bg,
                        " RECAL" if photometric.is_recalibrating else "",
                        last_mem,
                        mem_min,
                        mem_max,
                        mem_drift_pct,
                    )
                )

            # 关键：在下一轮 snapshot 前释放本轮 Image 引用。Python 赋值会先
            # 执行右侧 camera.read_algo_frame()，若旧 img 仍被局部变量持有，
            # K230 CHN1 的 VB buffer 紧张时可能触发 snapshot failed(3)。
            img = None

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
