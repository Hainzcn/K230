"""K230 视觉循迹主入口（A 采集 + B 多扫描带 + C IPM/RANSAC/EMA + D UART 通讯）。

本文件**只负责装配**，不写算法细节（plan §11.3）。

阶段 A（继承）：双通道 sensor + 显示 + OSD 叠加 + 资源保护 + 可选采样。

阶段 B（继承）：``vision.photometric`` bootstrap + 漂移监测；
``vision.line_detector`` 每帧 L0 + (可选 L1) + L2 → 5 条扫描带 cx / mass /
width / valid + Q_L2 评分。

阶段 C（继承）：
- ``vision.ground_mapper.GroundMapper`` IPM 单应矩阵，``calib.json`` 缺失时
  用安装几何推导占位 H（OSD 标 ``CALIB:DEFAULT``）。
- RANSAC 圆弧拟合 + LSQ 直线 fallback → ``e_y_mm`` / ``ψ_e_mrad``。
- ``vision.estimator.PathErrorEstimator`` EMA (α=0.5) + 符号防抖 (3 帧)。
- ``vision.quality.compute_q_full`` plan §6.6 全权重 Q。

阶段 D（新增）：
- ``comms.ImuLink``：UART(1, 115200) 直接读 MS901M 200 Hz 姿态数据，
  供云台前馈补偿（Stage E 起正式消费）。
- ``comms.McuLink``：UART(2, 921600) 双向通讯，40 Hz 发 MOTION_CMD，
  400 ms 心跳，500 ms 超时降级，OSD / 日志追加 MCU 在线状态。
- ``config.COMMS_ENABLE=False`` 时全部 UART 跳过（bench 调试兼容）。

后续阶段（E 起）将追加：``controller``（前馈 + 反馈 + 限幅）。
"""

import gc
import os
import sys
import time

import config
from vision.camera import Camera
from vision.gpio_button import GpioButton
from vision.photometric import Photometric
from vision.line_detector import LineDetector
from vision.quality import grade as q_grade, compute_q_full
from vision.ground_mapper import GroundMapper
from vision.geometry import (
    fit_circle_ransac,
    fit_line_lsq,
    compute_path_errors_arc,
    compute_path_errors_line,
)
from vision.estimator import PathErrorEstimator
from vision.debug_overlay import PathOverlayInfo
from comms.uart_link import ImuLink, McuLink


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


def _format_band_diag(detection):
    """把 5 条扫描带的 L2 结果压成一行，便于板端日志排查硬约束。"""
    parts = []
    for b in detection.bands:
        parts.append(
            "%d:m=%.0f,w=%d,cx=%.1f,%s"
            % (
                b.idx,
                b.mass,
                b.width_px,
                b.cx_px,
                "ok" if b.valid else (b.reject or "invalid"),
            )
        )
    return "; ".join(parts)


def _mem_snapshot():
    """返回 MicroPython 堆的 (free, alloc, total)，不是整机 1GB DDR/MMZ。"""
    free = gc.mem_free()
    try:
        alloc = gc.mem_alloc()
    except Exception:
        alloc = -1
    total = free + alloc if free >= 0 and alloc >= 0 else -1
    return free, alloc, total


def _setup_comms(cfg):
    """初始化双路 UART 链路。

    ``cfg.COMMS_ENABLE=False`` 时直接返回 (None, None)（bench 调试用）。
    任何初始化异常都捕获并打印，返回 (None, None) 不中断启动。
    """
    if not getattr(cfg, "COMMS_ENABLE", False):
        print("[VLT] COMMS_ENABLE=False, skipping UART init (bench mode)")
        return None, None
    try:
        from comms.ms901m import MS901MParser
        from comms.frame   import MCUFrameParser
        if not MS901MParser.self_test():
            print("[VLT] MS901MParser self_test FAILED; comms disabled")
            return None, None
        if not MCUFrameParser.self_test():
            print("[VLT] MCUFrameParser self_test FAILED; comms disabled")
            return None, None
        imu_link = ImuLink(
            uart_id = getattr(cfg, "IMU_UART_ID", 1),
            tx_io   = getattr(cfg, "IMU_UART1_TX_IO", 3),
            rx_io   = getattr(cfg, "IMU_UART1_RX_IO", 4),
        )
        mcu_link = McuLink(
            uart_id    = getattr(cfg, "MCU_UART_ID", 2),
            timeout_ms = getattr(cfg, "MCU_TIMEOUT_MS", 500),
            tx_io      = getattr(cfg, "MCU_UART2_TX_IO", 5),
            rx_io      = getattr(cfg, "MCU_UART2_RX_IO", 6),
        )
        print("[VLT] comms init OK (IMU UART%d, MCU UART%d)"
              % (getattr(cfg, "IMU_UART_ID", 1), getattr(cfg, "MCU_UART_ID", 2)))
        return imu_link, mcu_link
    except Exception as e:
        print("[VLT] comms init failed: %s" % e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
        return None, None


def main():
    print(
        "[VLT] vision_line_tracking start, config=%s, debug=%s, profile=%s"
        % (config.CONFIG_VERSION, config.DEBUG_DISPLAY,
           getattr(config, "LINE_DETECTION_PROFILE", "track"))
    )
    print(
        "[VLT] L2 thresholds: MIN_MASS=%s W=[%s..%s] COL_THR=%d Δcx_max=%d"
        % (config.MIN_MASS_PER_BAND,
           config.W_MIN_PX_PER_BAND, config.W_MAX_PX_PER_BAND,
           config.COL_SUM_THR_FOR_WIDTH, config.DELTA_CX_MAX_PX)
    )
    print(
        "[VLT] L2 segment select: gap_tol=%d prior_radius=%.1f prior_age_max=%d"
        % (config.get("LINE_RUN_GAP_TOLERANCE_PX", 0),
           config.get("LINE_CX_PRIOR_RADIUS_PX", config.DELTA_CX_MAX_PX),
           config.get("LINE_CX_PRIOR_AGE_MAX_FRAMES", 5))
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
    binary_button = GpioButton(config, name="binary_overlay")

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

    # 阶段 C：IPM + 几何 + EMA 装配（plan §5.2 + §6.3 + §7.4）
    calib = config.load_calibration()
    mapper = GroundMapper()
    calib_mode = mapper.load(calib)
    print("[VLT] IPM mode: %s%s" % (
        calib_mode,
        ("  err=" + mapper.load_error()) if calib_mode == "none" else "",
    ))
    if calib_mode != "none" and not mapper.self_test():
        print("[VLT] IPM self_test FAILED; demote to NO CALIB")
        calib_mode = "none"
    if calib_mode == "default":
        print("[VLT] CALIB:DEFAULT — e_y/ψ_e values carry tens-of-mm system bias.")
        print("[VLT]   to fix: run tools/calibrate_ipm.py on PC, copy calib.json to /sdcard/")
    estimator = PathErrorEstimator(config)
    path_overlay = PathOverlayInfo()
    path_overlay.calib_mode = calib_mode
    path_overlay.mapper = mapper if calib_mode != "none" else None

    # 阶段 D：UART 通讯初始化（COMMS_ENABLE=False 时返回 None, None）
    imu_link, mcu_link = _setup_comms(config)

    # 内存监测：泄漏看 free 的震荡；OSD 展示用 alloc/total，避免把 free 误读成占用。
    mem0_free, mem0_alloc, mem0_total = _mem_snapshot()
    mem_min = mem0_free
    mem_max = mem0_free
    last_mem_free = mem0_free
    last_mem_alloc = mem0_alloc
    last_mem_total = mem0_total

    last_log_ms = time.ticks_ms()
    snapshot_fail_streak = 0
    detection = None
    img = None

    # 阶段 D：通讯调度时间戳与状态缓存
    last_cmd_ms = time.ticks_ms()
    last_hb_ms  = time.ticks_ms()
    mcu_online  = False   # 上一次 is_online 结果（用于 OSD 避免每帧求值）
    # 阶段 C 每帧维护的最新几何状态（在 detector.process 之后填充，所有
    # render_overlay / 5s 日志路径都消费同一份）。
    arc_result = None
    line_result = None
    arc_mode = "none"
    e_y_filt_mm = 0.0
    psi_e_filt_mrad = 0.0
    e_y_age = 0
    psi_e_age = 0
    q_full = 0.0
    # 文字 / FPS / 内存等行 1Hz 由 maybe_update_fps 触发更新；
    # binary overlay 由 maybe_update_binary 独立频率触发。
    # render_overlay 始终用最新 detection + 最近一次缓存的 lines 整体重画，
    # 这样高频 binary 刷新不会让文字行抖动。
    cached_lines = []
    binary_event_text = None
    binary_event_until_ms = 0
    overlay_lines = cached_lines

    try:
        while True:
            os.exitpoint()
            # ticks_ms 放在循环顶部，即使 snapshot 持续失败也能让日志块按
            # LOG_INTERVAL_MS 节流地报警，不至于静默卡死。
            now = time.ticks_ms()
            overlay_rendered = False
            if (
                binary_event_text is not None
                and time.ticks_diff(binary_event_until_ms, now) <= 0
            ):
                binary_event_text = None
                overlay_lines = cached_lines

            # 阶段 D：drain 两路 UART（在 snapshot 前做，利用 snapshot 阻塞
            # 期间 UART 缓冲积累的数据，降低端到端延迟）。
            if imu_link is not None:
                imu_link.drain()
            if mcu_link is not None:
                mcu_link.drain(now)
                mcu_online = mcu_link.is_online(now)

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

            # 阶段 C 主算法：IPM → RANSAC 圆 / LSQ 直线 fallback → EMA → Q_full
            arc_result = None
            line_result = None
            arc_mode = "none"
            err_valid = False
            e_y_raw_mm = 0.0
            psi_e_raw_mrad = 0.0
            if calib_mode != "none":
                gp_list = mapper.bands_to_ground(detection.bands)
                valid_pts = [
                    (gp.x_g_mm, gp.y_g_mm) for gp in gp_list if gp.valid
                ]
                if len(valid_pts) >= config.RANSAC_MIN_SAMPLES:
                    arc_result = fit_circle_ransac(valid_pts, config)
                    if arc_result.succeeded:
                        e_y_raw_mm, psi_e_raw_mrad, err_valid = (
                            compute_path_errors_arc(arc_result, config)
                        )
                        arc_mode = "ransac"
                    else:
                        line_result = fit_line_lsq(valid_pts)
                        if line_result.succeeded:
                            e_y_raw_mm, psi_e_raw_mrad, err_valid = (
                                compute_path_errors_line(line_result, config)
                            )
                            arc_mode = "lsq"
                # else: 样本不足 → err_valid=False，下面 EMA 走 decay 分支
            # EMA + 符号防抖：valid=False 时只衰减不喂新观测
            e_y_filt_mm, psi_e_filt_mrad, e_y_age, psi_e_age = estimator.update(
                e_y_raw_mm, psi_e_raw_mrad, err_valid
            )
            # Q_full：arc 缺失时 q_geom / q_r_prior=0；调用方仍可用 detection.q_l2 看 L2 子分。
            q_full = compute_q_full(detection, arc_result, config)

            # 填阶段 C 几何 OSD 状态（render_overlay 内部使用）
            path_overlay.calib_mode = calib_mode
            path_overlay.mapper = mapper if calib_mode != "none" else None
            path_overlay.arc_mode = arc_mode
            path_overlay.valid = err_valid
            path_overlay.e_y_filt_mm = e_y_filt_mm
            path_overlay.psi_e_filt_mrad = psi_e_filt_mrad
            if arc_result is not None and arc_result.succeeded:
                path_overlay.arc_xc = arc_result.xc
                path_overlay.arc_yc = arc_result.yc
                path_overlay.arc_R = arc_result.R
                path_overlay.inlier_count = arc_result.inlier_count
                path_overlay.sample_count = arc_result.sample_count
            elif line_result is not None and line_result.succeeded:
                path_overlay.line_cx = line_result.cx
                path_overlay.line_cy = line_result.cy
                path_overlay.line_tx = line_result.tx
                path_overlay.line_ty = line_result.ty
                path_overlay.inlier_count = 0
                path_overlay.sample_count = line_result.sample_count
            else:
                path_overlay.inlier_count = 0
                path_overlay.sample_count = 0

            # 阶段 D：定时发送 MOTION_CMD（40 Hz）与 HEARTBEAT_K230（~2.5 Hz）
            if mcu_link is not None:
                if time.ticks_diff(now, last_cmd_ms) >= config.CMD_SEND_INTERVAL_MS:
                    last_cmd_ms = now
                    # Stage E 前 target_v / omega 由 config 占位值给出（均为 0）；
                    # Stage E 控制律落地后替换为控制律输出。
                    target_v     = getattr(config, "MOTION_DEFAULT_V", 0)
                    target_omega = getattr(config, "MOTION_DEFAULT_OMEGA", 0)
                    # 低电量减速：bat_mv 低于阈值时限制 target_v
                    bat_mv   = mcu_link.vehicle_bat_mv
                    bat_degrade_mv  = getattr(config, "BAT_DEGRADE_MV", 9500)
                    bat_degrade_max = getattr(config, "BAT_DEGRADE_V_MAX", 200)
                    if bat_mv > 0 and bat_mv < bat_degrade_mv:
                        if abs(target_v) > bat_degrade_max:
                            target_v = bat_degrade_max if target_v > 0 else -bat_degrade_max
                    mcu_link.send_motion(target_v, target_omega)

                if time.ticks_diff(now, last_hb_ms) >= config.HB_SEND_INTERVAL_MS:
                    last_hb_ms = now
                    mcu_link.send_heartbeat(now)

            if binary_button.poll(now):
                enabled = not camera.binary_overlay_enabled()
                if camera.set_binary_overlay_enabled(enabled):
                    binary_event_text = "BTN BIN %s" % ("ON" if enabled else "OFF")
                    binary_event_until_ms = time.ticks_add(
                        now, config.OSD_REFRESH_INTERVAL_MS
                    )
                    overlay_lines = list(cached_lines)
                    overlay_lines.append((binary_event_text, config.OSD_ALERT_COLOR))
                    camera.render_overlay(
                        overlay_lines, detection=detection, path=path_overlay
                    )
                    overlay_rendered = True

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

            # ---- 1Hz 文字行更新（FPS / Q / 内存 / 标定状态）---- #
            if camera.maybe_update_fps(now):
                cur_mem_free, cur_mem_alloc, cur_mem_total = _mem_snapshot()
                if cur_mem_free < mem_min:
                    mem_min = cur_mem_free
                if cur_mem_free > mem_max:
                    mem_max = cur_mem_free
                last_mem_free = cur_mem_free
                last_mem_alloc = cur_mem_alloc
                last_mem_total = cur_mem_total

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

                # 阶段 C 起 OSD line 2 改为 Q_full + 有效带数 + thr（cxN 信息
                # 移到阶段 C 的 e_y 行，避免 Q 行长度溢出 OSD）。Q_full 在 calib
                # 缺失时退化为 Q_L2 子集（geom + r_prior 项=0，自然降级到 lost）。
                q_for_osd = q_full if calib_mode != "none" else detection.q_l2
                q_color = None
                if q_for_osd < config.Q_HOLD:
                    q_color = config.OSD_ALERT_COLOR
                lines.append((
                    "Q %5.1f  V %d/%d  thr %d"
                    % (
                        q_for_osd,
                        detection.n_valid,
                        config.BAND_COUNT,
                        detection.threshold_used,
                    ),
                    q_color,
                ))

                # 阶段 C 路径误差行：calib_mode=="none" 时显示红色 NO CALIB；
                # 否则显示 e_y / ψ_e / R̂ / inliers / arc_mode。
                if calib_mode == "none":
                    lines.append((
                        "NO CALIB  (run tools/calibrate_ipm.py)",
                        config.OSD_NO_CALIB_COLOR,
                    ))
                else:
                    if not path_overlay.valid:
                        path_text = (
                            "e_y --     ψ --       R̂ --   in 0/%d  arc:%s"
                            % (path_overlay.sample_count, arc_mode)
                        )
                        path_color = config.OSD_ALERT_COLOR
                    elif arc_mode == "ransac":
                        path_text = (
                            "e_y %+5.0f mm  ψ %+5.0f mr  R̂ %3.0f mm  in %d/%d  arc:ransac"
                            % (
                                e_y_filt_mm, psi_e_filt_mrad,
                                path_overlay.arc_R,
                                path_overlay.inlier_count,
                                path_overlay.sample_count,
                            )
                        )
                        path_color = None
                    else:  # lsq
                        path_text = (
                            "e_y %+5.0f mm  ψ %+5.0f mr  R̂ inf      n=%d  arc:lsq"
                            % (
                                e_y_filt_mm, psi_e_filt_mrad,
                                path_overlay.sample_count,
                            )
                        )
                        path_color = None
                    lines.append((path_text, path_color))
                    if calib_mode == "default":
                        lines.append((
                            "CALIB:DEFAULT (estimate, run calibrate_ipm.py)",
                            config.OSD_CALIB_DEFAULT_COLOR,
                        ))

                # 阶段 D：MCU 在线状态行（COMMS_ENABLE=False 时跳过）
                if mcu_link is not None:
                    bat_mv_now = mcu_link.vehicle_bat_mv
                    mcu_status_text = "MCU:%s  bat:%dmV  cps:%+d" % (
                        "ON " if mcu_online else "OFF",
                        bat_mv_now,
                        mcu_link.vehicle_avg_cps,
                    )
                    mcu_color = None if mcu_online else config.OSD_ALERT_COLOR
                    lines.append((mcu_status_text, mcu_color))

                if photometric.is_recalibrating:
                    lines.append((
                        "PHOTO recal %d/%d"
                        % (
                            photometric.recalib_counter,
                            config.PHOTO_BOOTSTRAP_FRAMES,
                        ),
                        config.OSD_ALERT_COLOR,
                        "left",
                    ))

                mem_alert = (
                    mem_drift_pct >= config.MEM_DRIFT_ALERT_PCT
                    or cur_mem_free < config.MEM_LOW_ALERT_BYTES
                )
                if mem_alert:
                    if cur_mem_alloc >= 0 and cur_mem_total > 0:
                        mem_text = (
                            "MEM used %d/%d KB  free %d KB  drift %.1f%%"
                            % (
                                cur_mem_alloc // 1024,
                                cur_mem_total // 1024,
                                cur_mem_free // 1024,
                                mem_drift_pct,
                            )
                        )
                    else:
                        mem_text = (
                            "MEM free %d KB  (drift %.1f%%)"
                            % (cur_mem_free // 1024, mem_drift_pct)
                        )
                    lines.append((
                        mem_text,
                        config.OSD_ALERT_COLOR,
                    ))

                if capture_enabled:
                    lines.append(
                        "CAP %d/%d"
                        % (capture_count, config.CAPTURE_MAX_SAMPLES)
                    )
                # 文字行刷新：缓存最新 lines，并立即重画一次 OSD（含 binary）。
                cached_lines = lines
                if binary_event_text is not None:
                    overlay_lines = list(cached_lines)
                    overlay_lines.append((binary_event_text, config.OSD_ALERT_COLOR))
                else:
                    overlay_lines = cached_lines
                camera.render_overlay(
                    overlay_lines, detection=detection, path=path_overlay
                )
                overlay_rendered = True
            # ---- binary overlay 独立频率刷新（默认每帧）---- #
            elif (
                not overlay_rendered
                and detection is not None
                and camera.maybe_update_binary(now)
            ):
                # 用 1Hz 缓存的 lines 整体重画，避免 OSD 文字行抖动 / 缺失；
                # binary overlay 内部只重画 ROI 内红色高亮 + 预览，开销
                # ~10-30ms，不会拖累 algo FPS 低于 sensor 上限。
                camera.render_overlay(
                    overlay_lines, detection=detection, path=path_overlay
                )

            # 控制台日志节流：完整指标依然落日志，便于离线分析（plan §13.1）。
            if (
                detection is not None
                and time.ticks_diff(now, last_log_ms) >= config.LOG_INTERVAL_MS
            ):
                last_log_ms = now
                mem_drift_pct = (
                    100.0 * (mem_max - mem_min) / mem_max if mem_max > 0 else 0.0
                )
                # 阶段 C 起 Q 分级以 Q_full 为准；calib 缺失时退回 Q_L2 以
                # 不让 Q_full 的 q_geom/q_r_prior=0 把日志一律打成 lost。
                q_for_log = q_full if calib_mode != "none" else detection.q_l2
                if arc_result is not None and arc_result.succeeded:
                    arc_R_str = "%.0f" % arc_result.R
                    in_str = "%d" % arc_result.inlier_count
                else:
                    arc_R_str = "  -"
                    in_str = "0"
                # 阶段 D：MCU 链路统计
                if mcu_link is not None:
                    mcu_good, mcu_bad = mcu_link.stats()
                    if imu_link is not None:
                        imu_good, imu_bad = imu_link.stats()
                    else:
                        imu_good = imu_bad = 0
                    mcu_log_str = (
                        " mcu=%s bat=%dmV cps=%+d "
                        "imu_g=%d/b=%d mcu_g=%d/b=%d"
                        % (
                            "ON" if mcu_online else "OFF",
                            mcu_link.vehicle_bat_mv,
                            mcu_link.vehicle_avg_cps,
                            imu_good,
                            imu_bad,
                            mcu_good,
                            mcu_bad,
                        )
                    )
                else:
                    mcu_log_str = ""
                print(
                    "[VLT] algo_fps=%.1f period=%.1fms frames=%d "
                    "Q=%.1f(%s) V=%d/%d cxN=%.1f cxF=%.1f thr=%d "
                    "e_y=%+.1fmm psi_e=%+.1fmrad R_hat=%s in=%s/%d arc=%s "
                    "calib=%s%s "
                    "mu=%.1f sig=%.1f%s%s "
                    "mem_free=%d mem_alloc=%d mem_total=%d "
                    "(free_min=%d free_max=%d drift=%.1f%%)"
                    % (
                        camera.algo_fps(),
                        camera.algo_period_ms(),
                        camera.frame_count(),
                        q_for_log,
                        q_grade(q_for_log),
                        detection.n_valid,
                        config.BAND_COUNT,
                        detection.cx_near_px,
                        detection.cx_far_px,
                        detection.threshold_used,
                        e_y_filt_mm,
                        psi_e_filt_mrad,
                        arc_R_str,
                        in_str,
                        path_overlay.sample_count,
                        arc_mode,
                        calib_mode,
                        " HOLD(age=%d)" % e_y_age if (e_y_age > 0 and calib_mode != "none") else "",
                        photometric.mu_bg,
                        photometric.sigma_bg,
                        " RECAL" if photometric.is_recalibrating else "",
                        mcu_log_str,
                        last_mem_free,
                        last_mem_alloc,
                        last_mem_total,
                        mem_min,
                        mem_max,
                        mem_drift_pct,
                    )
                )
                if detection.n_valid == 0 or detection.cx_near_px < 0:
                    print(
                        "[VLT.band] mass_total=%.0f fg≈%.0f bands: %s"
                        % (
                            detection.mass_total,
                            detection.mass_total / 255.0,
                            _format_band_diag(detection),
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
        # 阶段 D：向 MCU 发一帧停止指令后再关闭（尽力而为，失败不阻塞）
        if mcu_link is not None:
            try:
                mcu_link.send_stop()
            except Exception:
                pass
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
            "[VLT] stopped, frames=%d, last_mem_free=%d, mem_range=%d~%d"
            % (
                camera.frame_count() if camera else -1,
                gc.mem_free(),
                mem_min,
                mem_max,
            )
        )


if __name__ == "__main__":
    main()
