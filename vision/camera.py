"""摄像头与显示封装（阶段 A）。

职责（plan §4 + §11.1）：

1. 配置 **同一颗 sensor 的两个通道**：
   - CHN0 ：YUV420SP 800×480，零拷贝绑定到 LCD VIDEO1 层做调试显示。
   - CHN1 ：GRAYSCALE 320×240，作为算法主输入（`snapshot` 拉取）。
2. 提供算法帧率统计（区别于 ``Display.fps()`` 的显示帧率）。
3. 提供 OSD 叠加：ROI 框、FPS、内存等调试信息（不每帧刷新）。
4. 提供静态样张采样接口（plan §12 阶段 A 任务："拍摄静态赛道样张 ≥ 100 张"）。
5. 提供严格的退出/清理顺序，避免 K230 端 MediaManager 句柄泄漏。

设计原则（plan §9.2）：

- 帧循环不创建 ``image.Image`` 对象；OSD buffer 在 :py:meth:`init` 中预分配。
- ``snapshot(chn=ALGO_CHN)`` 返回的 ``image.Image`` 由 sensor 层零拷贝管理，
  本类不做 ``.copy()``。
"""

import time

import image

from media.sensor import (
    Sensor,
    CAM_CHN_ID_0,
    CAM_CHN_ID_1,
)
from media.display import Display
from media.media import MediaManager

import config
from vision.interrupts import reraise_if_stop


_PIXFORMAT_NAME_TO_ENUM = {
    "YUV420SP": Sensor.YUV420SP,
    "RGB888": Sensor.RGB888,
    "RGB565": Sensor.RGB565,
    "GRAYSCALE": Sensor.GRAYSCALE,
}

_CHN_ID = {0: CAM_CHN_ID_0, 1: CAM_CHN_ID_1}


class Camera:
    """双通道 sensor + LCD 显示封装。

    使用模式（参见 ``vision_line_tracking.py``）::

        cam = Camera()
        cam.init()
        cam.start()
        try:
            while True:
                img = cam.read_algo_frame()
                ...
                if cam.maybe_update_fps():
                    cam.render_overlay([...])
        finally:
            cam.stop()
    """

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else config

        self._sensor = None
        self._osd = None
        self._media_inited = False
        self._display_inited = False
        self._sensor_running = False

        self._algo_chn = _CHN_ID[self.cfg.ALGO_CHN]
        self._display_chn = _CHN_ID[self.cfg.DISPLAY_CHN]

        # 帧统计
        self._frame_count = 0
        self._frames_in_window = 0
        self._algo_fps = 0.0
        self._last_fps_update_ms = 0

        # ROI 在显示坐标下的等比例位置（plan §4.3）
        self._roi_disp_total = self._roi_algo_to_display(self.cfg.ROI_TOTAL_PX)
        self._roi_disp_near = self._roi_algo_to_display(self.cfg.ROI_NEAR_PX)
        self._roi_disp_mid = self._roi_algo_to_display(self.cfg.ROI_MID_PX)
        self._roi_disp_far = self._roi_algo_to_display(self.cfg.ROI_FAR_PX)

        # 二值图叠加（plan §12 阶段 B 调试）。
        # K230 上 ``draw_image`` 对 GRAYSCALE source / mask / 缩放混合的组合会
        # 出现"调用成功但不显示"的静默失败；这里不再走 blit/mask 路径，而是在
        # OSD 上直接按 ``detection.binary_np`` 的前景连续段画线段/小矩形。
        self._binary_overlay_enabled = (
            getattr(self.cfg, "DEBUG_SHOW_BINARY", False)
            and self.cfg.DEBUG_DISPLAY
        )
        # 右上角预览独立开关：默认随 DEBUG_SHOW_BINARY，但用户可通过
        # ``DEBUG_SHOW_BINARY_PREVIEW=False`` 单独关掉预览（保留主 overlay）。
        self._binary_preview_enabled = (
            getattr(self.cfg, "DEBUG_SHOW_BINARY_PREVIEW", True)
            and getattr(self.cfg, "DEBUG_SHOW_BINARY", False)
            and self.cfg.DEBUG_DISPLAY
        )
        # overlay 渲染模式：见 config.OSD_BINARY_OVERLAY_MODE 注释。
        mode = str(getattr(self.cfg, "OSD_BINARY_OVERLAY_MODE", "bands_only"))
        # 兼容：旧名 "full_dither" → "full_dither_50"
        if mode == "full_dither":
            mode = "full_dither_50"
        valid_modes = (
            "bands_only", "full_solid",
            "full_dither_50", "full_dither_25", "full_dither_12",
        )
        if mode not in valid_modes:
            print("[camera] unknown OSD_BINARY_OVERLAY_MODE=%r, fallback to bands_only" % mode)
            mode = "bands_only"
        self._binary_overlay_mode = mode
        # 二值 overlay 刷新间隔（ms）。0 = 每帧；正数 = 节流。
        self._binary_refresh_ms = max(0, int(getattr(self.cfg, "OSD_BINARY_REFRESH_MS", 0)))
        self._last_binary_refresh_ms = 0
        self._binary_dest_x = 0
        self._binary_dest_y = 0
        self._binary_scale_x = 1.0
        self._binary_scale_y = 1.0
        # 调试：右上角原尺寸黑白预览的目标坐标（已知最简通路；无 mask 无缩放）
        self._binary_preview_x = 0
        self._binary_preview_y = 0
        # 节流：只在前 N 次 OSD 刷新打印诊断日志，避免长时间运行时刷屏。
        self._binary_dbg_remaining = 0
        # sensor.snapshot() 在 K230 上偶发 RuntimeError（如 failed(3)）时按丢帧处理。
        self._snapshot_error_log_remaining = 5

    # ------------------------------------------------------------------ #
    # 诊断
    # ------------------------------------------------------------------ #
    def log_configured_modes(self):
        """打印本次请求的 sensor mode 与各通道实际输出尺寸。

        通道编号取自 config（``DISPLAY_CHN``/``ALGO_CHN``），不是 hardcode 的
        "CHN0/CHN1"——swap 时也不会失真。
        """
        if self._sensor is None:
            return
        print(
            "[camera] request: sensor=%dx%d@%d, "
            "display→CHN%d=%s, algo→CHN%d=%s"
            % (
                self.cfg.SENSOR_REQ_WIDTH,
                self.cfg.SENSOR_REQ_HEIGHT,
                self.cfg.SENSOR_NOMINAL_FPS,
                self.cfg.DISPLAY_CHN,
                self.cfg.DISPLAY_PIXFORMAT,
                self.cfg.ALGO_CHN,
                self.cfg.ALGO_PIXFORMAT,
            )
        )
        for role, chn in (("display", self._display_chn),
                          ("algo   ", self._algo_chn)):
            try:
                print(
                    "[camera] CHN%d (%s): %dx%d"
                    % (
                        self.cfg.DISPLAY_CHN if role.strip() == "display"
                        else self.cfg.ALGO_CHN,
                        role,
                        self._sensor.width(chn=chn),
                        self._sensor.height(chn=chn),
                    )
                )
            except Exception as e:
                print("[camera] query %s failed: %s" % (role, e))

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def init(self):
        """构造 sensor、绑定 CHN0 到 LCD、配置 CHN1 算法通道、初始化 Display + Media。

        遵守官方推荐顺序（参考 camera_single_bind_lcd.py 与 plan §11）：
            Sensor() → reset → set_framesize/pixformat (CHN0) → bind_layer
            → set_framesize/pixformat (CHN1) → Display.init → MediaManager.init

        必须**三件套一起**传给 ``Sensor(width=, height=, fps=)``。只传 fps
        时驱动会保留默认 1920×1080，而 OV5647 在 1920×1080 下最高 30 FPS
        （其 60 FPS 只支持 ≤ 1280×720），期望的 fps 会被静默忽略，表现为
        algo_fps 顽固地停在 30.0 不动。
        """
        self._sensor = Sensor(
            width=self.cfg.SENSOR_REQ_WIDTH,
            height=self.cfg.SENSOR_REQ_HEIGHT,
            fps=self.cfg.SENSOR_NOMINAL_FPS,
        )
        self._sensor.reset()
        self._sensor.set_hmirror(True)
        self._sensor.set_vflip(True)

        # ---- CHN0：显示通道 ----
        self._sensor.set_framesize(
            chn=self._display_chn,
            width=self.cfg.DISPLAY_WIDTH,
            height=self.cfg.DISPLAY_HEIGHT,
        )
        self._sensor.set_pixformat(
            _PIXFORMAT_NAME_TO_ENUM[self.cfg.DISPLAY_PIXFORMAT],
            chn=self._display_chn,
        )
        bind_info = self._sensor.bind_info(chn=self._display_chn)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

        # ---- CHN1：算法通道 ----
        self._sensor.set_framesize(
            chn=self._algo_chn,
            width=self.cfg.ALGO_WIDTH,
            height=self.cfg.ALGO_HEIGHT,
        )
        self._sensor.set_pixformat(
            _PIXFORMAT_NAME_TO_ENUM[self.cfg.ALGO_PIXFORMAT],
            chn=self._algo_chn,
        )

        # ---- Display ----
        display_type = getattr(Display, self.cfg.DISPLAY_TYPE)
        Display.init(
            display_type,
            width=self.cfg.DISPLAY_WIDTH,
            height=self.cfg.DISPLAY_HEIGHT,
            to_ide=self.cfg.DISPLAY_TO_IDE,
        )
        self._display_inited = True

        MediaManager.init()
        self._media_inited = True

        # ---- OSD 缓冲 ----
        if self.cfg.DEBUG_DISPLAY:
            self._osd = image.Image(
                self.cfg.DISPLAY_WIDTH,
                self.cfg.DISPLAY_HEIGHT,
                image.ARGB8888,
            )
            self._osd.clear()

        # ---- 二值图叠加：计算坐标（不再预分配 blit/mask 中间图）---- #
        if self._binary_overlay_enabled or self._binary_preview_enabled:
            try:
                roi_x, roi_y, roi_w, roi_h = self.cfg.ROI_TOTAL_PX
                sx = self.cfg.DISPLAY_WIDTH / float(self.cfg.ALGO_WIDTH)
                sy = self.cfg.DISPLAY_HEIGHT / float(self.cfg.ALGO_HEIGHT)
                self._binary_dest_x = int(roi_x * sx)
                self._binary_dest_y = int(roi_y * sy)
                self._binary_scale_x = sx
                self._binary_scale_y = sy
                # 右上角预览窗坐标：x=DISPLAY_W-roi_w-10, y=10
                self._binary_preview_x = max(
                    0, self.cfg.DISPLAY_WIDTH - roi_w - 10
                )
                self._binary_preview_y = 10
                # 前 10 次 OSD 刷新打印诊断；之后静默
                self._binary_dbg_remaining = 10
                print(
                    "[camera] binary overlay setup: overlay=%s preview=%s "
                    "mode=%s roi=%dx%d dest=(%d,%d) scale=(%.2f,%.2f) preview=(%d,%d)"
                    % (
                        self._binary_overlay_enabled,
                        self._binary_preview_enabled,
                        self._binary_overlay_mode,
                        roi_w, roi_h,
                        self._binary_dest_x, self._binary_dest_y,
                        sx, sy,
                        self._binary_preview_x, self._binary_preview_y,
                    )
                )
            except Exception as e:
                print("[camera] binary overlay setup failed:", e)
                self._binary_overlay_enabled = False
                self._binary_preview_enabled = False

        return self

    def start(self):
        """启动 sensor 数据流。MediaManager.init 之后调用（参见 Sensor 文档）。"""
        if self._sensor is None:
            raise RuntimeError("Camera.init() must be called before start()")
        self._sensor.run()
        self._sensor_running = True
        self._last_fps_update_ms = time.ticks_ms()
        return self

    def stop(self):
        """严格逆序释放：sensor.stop → Display.deinit → MediaManager.deinit。

        每个步骤都被保护，确保即使前一步抛异常也不会导致后续资源泄漏。
        """
        try:
            if self._sensor_running and isinstance(self._sensor, Sensor):
                self._sensor.stop()
        except Exception as e:
            print("[camera] sensor.stop failed:", e)
        finally:
            self._sensor_running = False

        if self._display_inited:
            try:
                Display.deinit()
            except Exception as e:
                print("[camera] Display.deinit failed:", e)
            finally:
                self._display_inited = False

        # 让底层有时间归还 buffer
        time.sleep_ms(50)

        if self._media_inited:
            try:
                MediaManager.deinit()
            except Exception as e:
                print("[camera] MediaManager.deinit failed:", e)
            finally:
                self._media_inited = False

    # ------------------------------------------------------------------ #
    # 帧读取
    # ------------------------------------------------------------------ #
    def read_algo_frame(self, timeout=None):
        """从 CHN1 拉一帧算法输入。返回 ``image.Image`` 或 ``None``。

        :param timeout: ``None`` 时使用 ``config.SNAPSHOT_TIMEOUT_MS``。
        """
        if timeout is None:
            timeout = self.cfg.SNAPSHOT_TIMEOUT_MS
        try:
            img = self._sensor.snapshot(chn=self._algo_chn, timeout=timeout)
        except RuntimeError as e:
            reraise_if_stop(e)
            if self._snapshot_error_log_remaining > 0:
                self._snapshot_error_log_remaining -= 1
                print("[camera] snapshot RuntimeError:", e)
            return None
        except Exception as e:
            reraise_if_stop(e)
            if self._snapshot_error_log_remaining > 0:
                self._snapshot_error_log_remaining -= 1
                print("[camera] snapshot failed:", e)
            return None
        if img is not None:
            self._frame_count += 1
            self._frames_in_window += 1
        return img

    # ------------------------------------------------------------------ #
    # FPS 统计
    # ------------------------------------------------------------------ #
    def algo_fps(self):
        """CHN1 ``snapshot()`` 拉帧的实际速率（float），**唯一可靠的帧率指标**。

        ``snapshot()`` 是阻塞拉帧：在阶段 A 主循环几乎不做事的前提下，
        这个值同时就是 sensor 对 CHN1 的实际供帧率——不需要另一路"流速率"
        来做分子分母，两者在这里本来就重合。

        注：``Display.fps()`` 返回的是 LCD VSync（ST7701 约 60 Hz 固定），
        不是 sensor 出帧率。前两轮把它当 stream FPS 是错的，已全部拆除。
        """
        return self._algo_fps

    def algo_period_ms(self):
        """算法侧平均帧周期（ms）= 1000 / algo_fps，``algo_fps=0`` 时返回 0。"""
        if self._algo_fps <= 0.0:
            return 0.0
        return 1000.0 / self._algo_fps

    def frame_count(self):
        return self._frame_count

    # ------------------------------------------------------------------ #
    # 启动期诊断：raw snapshot 计时
    # ------------------------------------------------------------------ #
    def probe_snapshot_timing(self, n_frames=60):
        """连续阻塞拉 ``n_frames`` 帧，统计每次 snapshot 的微秒耗时分布。

        用于把"snapshot 自身阻塞时间"和"主循环其他开销（exitpoint / Python /
        IDE 通讯）"分开诊断：

        - 若 snapshot 单次平均 ≈ 16~17 ms（60 FPS 周期），主循环额外 ~13 ms
          来自 Python/IDE 路径——优化点在主循环外。
        - 若 snapshot 单次平均 ≈ 30 ms（30 FPS 周期），CHN1 管线本身限速，
          软件层面没法再压；只能换通道或接受现状。

        探针不计入 ``frame_count`` / ``algo_fps`` 统计；执行完毕主循环再开始。
        """
        if self._sensor is None or n_frames <= 0:
            return
        samples = []
        # 先取 1 帧丢弃，避免首帧冷启动偏差污染统计。
        try:
            self._sensor.snapshot(chn=self._algo_chn,
                                  timeout=self.cfg.SNAPSHOT_TIMEOUT_MS)
        except Exception as e:
            reraise_if_stop(e)
            pass
        for _ in range(n_frames):
            t0 = time.ticks_us()
            img = self._sensor.snapshot(
                chn=self._algo_chn,
                timeout=self.cfg.SNAPSHOT_TIMEOUT_MS,
            )
            dt = time.ticks_diff(time.ticks_us(), t0)
            if img is not None:
                samples.append(dt)
            img = None
        if not samples:
            print("[camera.probe] no samples collected")
            return
        samples.sort()
        n = len(samples)
        avg = sum(samples) / n
        p50 = samples[n // 2]
        p95 = samples[min(n - 1, int(n * 0.95))]
        print(
            "[camera.probe] snapshot x%d  min=%dus  p50=%dus  avg=%.0fus  "
            "p95=%dus  max=%dus  → fps_eq=%.1f"
            % (n, samples[0], p50, avg, p95, samples[-1], 1e6 / avg)
        )

    def maybe_update_fps(self, now_ms=None):
        """1 秒内累计的算法帧数换算为 FPS。每 ``OSD_REFRESH_INTERVAL_MS`` 触发一次。"""
        now = now_ms if now_ms is not None else time.ticks_ms()
        elapsed = time.ticks_diff(now, self._last_fps_update_ms)
        if elapsed >= self.cfg.OSD_REFRESH_INTERVAL_MS:
            self._algo_fps = self._frames_in_window * 1000.0 / elapsed
            self._frames_in_window = 0
            self._last_fps_update_ms = now
            return True
        return False

    def maybe_update_binary(self, now_ms=None):
        """二值 overlay 是否到了刷新时刻（独立于 FPS 文字行）。

        ``OSD_BINARY_REFRESH_MS=0`` 时永远返回 True（每帧刷新）；正数时按
        该 ms 间隔节流。返回 True 表示主循环应当调用 :meth:`render_overlay`
        把最新 ``detection`` 画上 OSD。
        """
        if not self.cfg.DEBUG_DISPLAY or self._osd is None:
            return False
        if not (self._binary_overlay_enabled or self._binary_preview_enabled):
            # 二值调试都关了，沿用 maybe_update_fps 的 1Hz 节流就够了。
            return False
        if self._binary_refresh_ms <= 0:
            return True
        now = now_ms if now_ms is not None else time.ticks_ms()
        elapsed = time.ticks_diff(now, self._last_binary_refresh_ms)
        if elapsed >= self._binary_refresh_ms:
            self._last_binary_refresh_ms = now
            return True
        return False

    def binary_overlay_enabled(self):
        """返回主画面 ROI 红色二值 overlay 当前运行期状态。"""
        return bool(self._binary_overlay_enabled)

    def set_binary_overlay_enabled(self, enabled):
        """运行期切换主画面 ROI 红色二值 overlay。"""
        next_enabled = bool(enabled) and self.cfg.DEBUG_DISPLAY
        if next_enabled == self._binary_overlay_enabled:
            return False
        self._binary_overlay_enabled = next_enabled
        self._last_binary_refresh_ms = 0
        print(
            "[camera] binary overlay %s"
            % ("ON" if self._binary_overlay_enabled else "OFF")
        )
        return True

    # ------------------------------------------------------------------ #
    # 调试叠加
    # ------------------------------------------------------------------ #
    def render_overlay(self, lines=None, detection=None):
        """重绘 OSD：3 段 ROI + 总 ROI 外框 + 若干行文本 + (可选) 检测可视化。

        plan §9.2 守则 7：仅在 ``maybe_update_fps`` 返回 ``True`` 时调用，
        不每帧刷新；正式比赛通过 ``DEBUG_DISPLAY=False`` 关闭。

        :param lines: 可迭代，每项可以是 ``str`` 或 ``(text, color)`` 二元组。
            - ``str``：使用 ``config.OSD_TEXT_COLOR`` 默认颜色；
            - ``(text, color)``：按给定 ``(R, G, B)`` 颜色；``color=None``
              时回退默认。告警类信息（丢帧率 / 低内存）应传红色，参见
              ``config.OSD_ALERT_COLOR``。
        :param detection: 可选 :class:`vision.line_detector.DetectionResult`。
            非空时画 5 条扫描带边框、每带 cx 圆点（valid 绿/invalid 红）、
            每带等效宽度水平线段。Q_L2 由调用方放进 ``lines`` 文本行。
        """
        if not self.cfg.DEBUG_DISPLAY or self._osd is None:
            return

        self._osd.clear()

        # ROI：总框为黄色粗线，三个子带为细线（颜色按近/中/远逐渐变浅）。
        self._draw_roi(self._roi_disp_total, self.cfg.OSD_ROI_COLOR,
                       self.cfg.OSD_ROI_THICKNESS)
        self._draw_roi(self._roi_disp_near, (0, 255, 0), 1)
        self._draw_roi(self._roi_disp_mid, (0, 200, 200), 1)
        self._draw_roi(self._roi_disp_far, (200, 100, 100), 1)

        if detection is not None:
            self._draw_detection(detection)

        if lines:
            x = 8
            y = 8
            font_h = self.cfg.OSD_TEXT_SIZE_PX
            default_color = self.cfg.OSD_TEXT_COLOR
            for item in lines:
                if isinstance(item, tuple):
                    text = item[0]
                    col = item[1] if len(item) > 1 and item[1] is not None else default_color
                else:
                    text = item
                    col = default_color
                self._osd.draw_string_advanced(x, y, font_h, text, color=col)
                y += font_h + 2

        Display.show_image(self._osd, x=0, y=0, layer=Display.LAYER_OSD0)

    def _draw_roi(self, rect, color, thickness):
        x, y, w, h = rect
        self._osd.draw_rectangle(x, y, w, h, color=color, thickness=thickness)

    def _draw_detection(self, detection):
        """把 :class:`DetectionResult` 中 5 条带的几何信息画到 OSD。

        绘制顺序（先底层后上层，避免几何标记被二值图盖住）：
        1. 二值图叠加（黑线像素 → 半透明红色色块），只在
           ``DEBUG_SHOW_BINARY=True`` 时执行；
        2. 5 条扫描带矩形边框；
        3. 每带 cx 圆点 / 等效宽度水平线段。

        几何信息全部位于算法分辨率坐标系（320×240），通过
        :meth:`algo_xy_to_display` 等比例换算到显示分辨率。
        """
        # ---- 1. 二值图叠加 ---- #
        self._draw_binary_debug(detection)

        bands = detection.bands
        if not bands:
            return
        band_color = self.cfg.OSD_BAND_COLOR
        valid_color = self.cfg.OSD_CX_VALID_COLOR
        invalid_color = self.cfg.OSD_CX_INVALID_COLOR
        width_color = self.cfg.OSD_WIDTH_COLOR
        radius = self.cfg.OSD_CX_RADIUS_PX

        for b in bands:
            # 扫描带矩形（使用 ROI 全宽，带高 = BAND_HEIGHT_PX）
            x0_d, y0_d = self.algo_xy_to_display(0, b.y_top)
            x1_d, y1_d = self.algo_xy_to_display(self.cfg.ALGO_WIDTH, b.y_bot)
            self._osd.draw_rectangle(
                x0_d,
                y0_d,
                max(1, x1_d - x0_d),
                max(1, y1_d - y0_d),
                color=band_color,
                thickness=1,
            )

            if b.cx_px < 0:
                continue

            cy = (b.y_top + b.y_bot) // 2
            cx_d, cy_d = self.algo_xy_to_display(b.cx_px, cy)
            color = valid_color if b.valid else invalid_color
            self._osd.draw_circle(
                cx_d, cy_d, radius, color=color, thickness=2, fill=b.valid
            )

            # 等效宽度：以 cx 为中心画一段水平线段，长度 = width_px
            if b.width_px > 0:
                half = b.width_px // 2
                wx0 = max(0, int(b.cx_px) - half)
                wx1 = min(self.cfg.ALGO_WIDTH - 1, int(b.cx_px) + half)
                wx0_d, wy_d = self.algo_xy_to_display(wx0, cy)
                wx1_d, _ = self.algo_xy_to_display(wx1, cy)
                self._osd.draw_line(
                    wx0_d, wy_d, wx1_d, wy_d, color=width_color, thickness=2
                )

    def _draw_binary_debug(self, detection):
        """绘制二值图调试 overlay。preview 与 overlay 各自独立。

        实现要点：

        1. ``bytes(binary_np)`` 一次物化为 Python bytes（48KB），后续走
           bytes 索引（~50ns），远快于 ulab ``row[x]`` 索引（~100us）。
        2. preview（右上角黑白小窗）由 ``DEBUG_SHOW_BINARY_PREVIEW``
           独立控制，可单独关闭以释放 OSD 区域。
        3. overlay（主画面 ROI 红色高亮）有三种模式：

           - ``bands_only``  ：仅 5 条 L2 扫描带，覆盖 ROI ~27%（旧默认）；
           - ``full_solid``  ：全 ROI 不透明红，连续无缝隙；
           - ``full_dither`` ：全 ROI 棋盘格抖动，1×1 红点伪 ~50% 透明度。

           K230 ``image.draw_rectangle`` 不支持 alpha，硬件 OSD 真半透明
           只能改 ARGB buffer，但前面记录有 cache 一致性 quirk，因此用
           dither 抖动模拟。
        """
        if not (self._binary_overlay_enabled or self._binary_preview_enabled):
            return
        binary_np = getattr(detection, "binary_np", None)
        if binary_np is None:
            return

        roi_x, roi_y_origin, roi_w, roi_h = self.cfg.ROI_TOTAL_PX
        del roi_x

        verbose = self._binary_dbg_remaining > 0
        if verbose:
            self._binary_dbg_remaining -= 1

        try:
            binary_bytes = bytes(binary_np)
        except Exception:
            try:
                binary_bytes = binary_np.tobytes()
            except Exception as e:
                reraise_if_stop(e)
                print("[camera.dbg] binary_np → bytes failed:", e)
                return

        if len(binary_bytes) < roi_w * roi_h:
            print(
                "[camera.dbg] binary bytes truncated: %d < %d, abort overlay"
                % (len(binary_bytes), roi_w * roi_h)
            )
            return

        preview_count = 0
        if self._binary_preview_enabled:
            preview_count = self._draw_preview(
                binary_bytes, roi_w, roi_h, detection.bands, roi_y_origin
            )

        overlay_count = 0
        overlay_mode = self._binary_overlay_mode
        if self._binary_overlay_enabled:
            try:
                if overlay_mode == "full_solid":
                    overlay_count = self._draw_overlay_full_solid(
                        binary_bytes, roi_w, roi_h
                    )
                elif overlay_mode == "full_dither_50":
                    overlay_count = self._draw_overlay_full_dither(
                        binary_bytes, roi_w, roi_h, divisor=2
                    )
                elif overlay_mode == "full_dither_25":
                    overlay_count = self._draw_overlay_full_dither(
                        binary_bytes, roi_w, roi_h, divisor=4
                    )
                elif overlay_mode == "full_dither_12":
                    overlay_count = self._draw_overlay_full_dither(
                        binary_bytes, roi_w, roi_h, divisor=8
                    )
                else:  # bands_only
                    overlay_count = self._draw_overlay_bands(
                        binary_bytes, roi_w, detection.bands, roi_y_origin
                    )
            except Exception as e:
                reraise_if_stop(e)
                print("[camera.dbg] overlay draw failed:", e)

        if verbose:
            band_area = self.cfg.BAND_HEIGHT_PX * self.cfg.BAND_COUNT * roi_w
            fg_band = int(detection.mass_total) // 255 if band_area else 0
            fg_pct = (100.0 * fg_band / band_area) if band_area else 0.0
            print(
                "[camera.dbg] binary thr=%d band_fg=%d/%d (%.1f%%) "
                "mode=%s preview=%d overlay=%d"
                % (
                    detection.threshold_used,
                    fg_band, band_area, fg_pct,
                    overlay_mode,
                    preview_count, overlay_count,
                )
            )

    # ------------------------------------------------------------------ #
    # 二值图绘制：preview / overlay 三种模式
    # ------------------------------------------------------------------ #
    def _draw_preview(self, binary_bytes, roi_w, roi_h, bands, roi_y_origin):
        """右上角黑白预览窗（原 ROI 尺寸）。仅扫 5 条带以省 CPU。"""
        try:
            self._osd.draw_rectangle(
                self._binary_preview_x, self._binary_preview_y,
                roi_w, roi_h,
                color=(0, 0, 0), thickness=1, fill=True,
            )
        except Exception as e:
            reraise_if_stop(e)
            print("[camera.dbg] preview bg failed:", e)

        stride_x = max(1, int(getattr(self.cfg, "OSD_BINARY_PREVIEW_STRIDE_X", 2)))
        stride_y = max(1, int(getattr(self.cfg, "OSD_BINARY_PREVIEW_STRIDE_Y", 2)))
        min_run_px = max(1, int(getattr(self.cfg, "OSD_BINARY_MIN_RUN_PX", 1)))

        spans = 0
        try:
            for band in bands:
                y0 = band.y_top - roi_y_origin
                y1 = band.y_bot - roi_y_origin
                if y0 < 0:
                    y0 = 0
                if y1 > roi_h:
                    y1 = roi_h
                if y1 <= y0:
                    continue
                for y in range(y0, y1, stride_y):
                    spans += self._draw_row_spans_from_bytes(
                        binary_bytes, y, roi_w,
                        self._binary_preview_x, self._binary_preview_y,
                        1.0, 1.0, (255, 255, 255),
                        stride_x, stride_y, True, min_run_px,
                    )
        except Exception as e:
            reraise_if_stop(e)
            print("[camera.dbg] preview band scan failed:", e)

        try:
            self._osd.draw_rectangle(
                self._binary_preview_x - 1, self._binary_preview_y - 1,
                roi_w + 2, roi_h + 2,
                color=(255, 255, 0), thickness=1,
            )
        except Exception as e:
            reraise_if_stop(e)
            print("[camera.dbg] preview frame failed:", e)
        return spans

    def _draw_overlay_bands(self, binary_bytes, roi_w, bands, roi_y_origin):
        """主画面 overlay：仅 5 条 L2 扫描带（旧默认）。"""
        stride_x = max(1, int(getattr(self.cfg, "OSD_BINARY_STRIDE_X", 1)))
        stride_y = max(1, int(getattr(self.cfg, "OSD_BINARY_STRIDE_Y", 2)))
        min_run_px = max(1, int(getattr(self.cfg, "OSD_BINARY_MIN_RUN_PX", 1)))
        roi_h = self.cfg.ROI_TOTAL_PX[3]
        color = self.cfg.OSD_BINARY_COLOR
        sx = self._binary_scale_x
        sy = self._binary_scale_y
        spans = 0
        for band in bands:
            y0 = band.y_top - roi_y_origin
            y1 = band.y_bot - roi_y_origin
            if y0 < 0:
                y0 = 0
            if y1 > roi_h:
                y1 = roi_h
            if y1 <= y0:
                continue
            for y in range(y0, y1, stride_y):
                spans += self._draw_row_spans_from_bytes(
                    binary_bytes, y, roi_w,
                    self._binary_dest_x, self._binary_dest_y,
                    sx, sy, color,
                    stride_x, stride_y, True, min_run_px,
                )
        return spans

    def _draw_overlay_full_solid(self, binary_bytes, roi_w, roi_h):
        """主画面 overlay：全 ROI 不透明红色，逐行合并连续段画矩形。"""
        color = self.cfg.OSD_BINARY_COLOR
        sx = self._binary_scale_x
        sy = self._binary_scale_y
        dest_x = self._binary_dest_x
        dest_y = self._binary_dest_y
        bb = binary_bytes
        osd_draw_rect = self._osd.draw_rectangle
        dh = max(1, int(sy))
        spans = 0
        for y in range(roi_h):
            row_offset = y * roi_w
            dy = dest_y + int(y * sy)
            run_start = -1
            for x in range(roi_w):
                if bb[row_offset + x] > 0:
                    if run_start < 0:
                        run_start = x
                elif run_start >= 0:
                    dx = dest_x + int(run_start * sx)
                    dw = max(1, int((x - run_start) * sx))
                    osd_draw_rect(dx, dy, dw, dh,
                                  color=color, thickness=1, fill=True)
                    spans += 1
                    run_start = -1
            if run_start >= 0:
                dx = dest_x + int(run_start * sx)
                dw = max(1, int((roi_w - run_start) * sx))
                osd_draw_rect(dx, dy, dw, dh,
                              color=color, thickness=1, fill=True)
                spans += 1
        return spans

    def _draw_overlay_full_dither(self, binary_bytes, roi_w, roi_h, divisor):
        """主画面 overlay：全 ROI 抖动模式画 1×1 红点。

        :param divisor: 红色密度倒数，覆盖率 = 1/divisor。
            - ``2``：(x+y)%2==0 棋盘格 → 50%（伪半透明）
            - ``4``：偶数行 + 偶数列 → 25%（更轻）
            - ``8``：偶数行 + 4 取 1 列 → 12.5%（最稀疏，CPU 最低）

        每 OSD 像素是 ``int(sx)×int(sy)``（典型 2×2），相邻位置不画，肉眼
        看就是均匀的红色滤镜叠加。覆盖完整连续，无几何"片状"或"竖缝"。
        """
        color = self.cfg.OSD_BINARY_COLOR
        sx = self._binary_scale_x
        sy = self._binary_scale_y
        dest_x = self._binary_dest_x
        dest_y = self._binary_dest_y
        bb = binary_bytes
        osd_draw_rect = self._osd.draw_rectangle
        dot_w = max(1, int(sx))
        dot_h = max(1, int(sy))
        spans = 0

        if divisor == 2:
            # 50% 棋盘格
            for y in range(roi_h):
                row_offset = y * roi_w
                dy = dest_y + int(y * sy)
                x_start = 1 if (y & 1) else 0
                for x in range(x_start, roi_w, 2):
                    if bb[row_offset + x] > 0:
                        dx = dest_x + int(x * sx)
                        osd_draw_rect(dx, dy, dot_w, dot_h,
                                      color=color, thickness=1, fill=True)
                        spans += 1
        elif divisor == 4:
            # 25%：偶数行 × 偶数列
            for y in range(0, roi_h, 2):
                row_offset = y * roi_w
                dy = dest_y + int(y * sy)
                for x in range(0, roi_w, 2):
                    if bb[row_offset + x] > 0:
                        dx = dest_x + int(x * sx)
                        osd_draw_rect(dx, dy, dot_w, dot_h,
                                      color=color, thickness=1, fill=True)
                        spans += 1
        else:
            # 12.5%（divisor==8 或其他兜底）：偶数行 × 4 取 1 列，
            # 奇 / 偶 step 的列起点交错以避免出现"竖排红线"。
            for y in range(0, roi_h, 2):
                row_offset = y * roi_w
                dy = dest_y + int(y * sy)
                x_start = 0 if ((y >> 1) & 1) == 0 else 2
                for x in range(x_start, roi_w, 4):
                    if bb[row_offset + x] > 0:
                        dx = dest_x + int(x * sx)
                        osd_draw_rect(dx, dy, dot_w, dot_h,
                                      color=color, thickness=1, fill=True)
                        spans += 1
        return spans

    def _draw_row_spans_from_bytes(
        self,
        binary_bytes,
        y,
        src_w,
        dest_x,
        dest_y,
        scale_x,
        scale_y,
        color,
        stride_x,
        stride_y,
        fill_rows,
        min_run_px,
    ):
        """从 binary_bytes 的第 y 行扫前景连续段并画到 OSD（bands_only / preview 用）。

        访问 ``binary_bytes`` 是纯 Python ``bytes`` 索引（~50ns），远快于
        ``binary_np[y][x]`` 这条 ulab 索引链（~50-100us）。每次 OSD
        刷新合计 ~12800 次字节索引，整体 < 5ms。
        """
        spans = 0
        row_offset = y * src_w
        run_start = -1
        x = 0
        bb = binary_bytes
        osd_draw_rect = self._osd.draw_rectangle
        dy = dest_y + int(y * scale_y)
        if fill_rows:
            dh = max(1, int(stride_y * scale_y))
        else:
            dh = 1
        while x < src_w:
            on = bb[row_offset + x] > 0
            if on:
                if run_start < 0:
                    run_start = x
            elif run_start >= 0:
                if x - run_start >= min_run_px:
                    dx = dest_x + int(run_start * scale_x)
                    dw = max(1, int((x - run_start) * scale_x))
                    osd_draw_rect(dx, dy, dw, dh, color=color, thickness=1, fill=True)
                    spans += 1
                run_start = -1
            x += stride_x
        if run_start >= 0:
            if src_w - run_start >= min_run_px:
                dx = dest_x + int(run_start * scale_x)
                dw = max(1, int((src_w - run_start) * scale_x))
                osd_draw_rect(dx, dy, dw, dh, color=color, thickness=1, fill=True)
                spans += 1
        return spans

    def algo_xy_to_display(self, x, y):
        """算法坐标 (px @ 320×240) → 显示坐标 (px @ DISPLAY_WIDTH × DISPLAY_HEIGHT)。

        用同一组比例 (DISPLAY_W / ALGO_W) 与 (DISPLAY_H / ALGO_H) 等比例缩放，
        与 :meth:`_roi_algo_to_display` 保持一致——两通道同 sensor，视野基本
        对齐，等比例换算足够调试用。
        """
        sx = self.cfg.DISPLAY_WIDTH / float(self.cfg.ALGO_WIDTH)
        sy = self.cfg.DISPLAY_HEIGHT / float(self.cfg.ALGO_HEIGHT)
        return int(x * sx), int(y * sy)

    # ------------------------------------------------------------------ #
    # 采样（plan §12 阶段 A："静态赛道样张 ≥ 100 张"）
    # ------------------------------------------------------------------ #
    def save_algo_frame(self, img, path, quality=None):
        """把 CHN1 帧落盘为 JPEG。返回布尔成功标志。

        K230 文件 IO 在 SD 卡上较慢，单帧约 30~80 ms；调用方应控制频率。
        """
        if img is None:
            return False
        if quality is None:
            quality = self.cfg.CAPTURE_JPEG_QUALITY
        try:
            img.save(path, quality=quality)
            return True
        except Exception as e:
            print("[camera] save frame failed:", path, e)
            return False

    # ------------------------------------------------------------------ #
    # 工具
    # ------------------------------------------------------------------ #
    def _roi_algo_to_display(self, roi_algo):
        """把算法坐标系下的 ROI 等比例缩放到显示坐标系。

        算法分辨率与显示分辨率比例不同（320×240 vs 800×480），但使用同一颗 sensor
        的不同输出通道，视野基本一致，等比例缩放足够调试用。
        """
        ax, ay, aw, ah = roi_algo
        sx = self.cfg.DISPLAY_WIDTH / float(self.cfg.ALGO_WIDTH)
        sy = self.cfg.DISPLAY_HEIGHT / float(self.cfg.ALGO_HEIGHT)
        return (int(ax * sx), int(ay * sy), int(aw * sx), int(ah * sy))

    # ------------------------------------------------------------------ #
    # 自检（plan §11.2 强制项）
    # ------------------------------------------------------------------ #
    def self_test(self):
        """阶段 A 自检：抓 5 帧算法输入，确认尺寸与像素格式符合配置。"""
        ok = True
        for _ in range(5):
            img = self.read_algo_frame()
            if img is None:
                print("[camera.self_test] snapshot failed")
                ok = False
                break
            if img.width() != self.cfg.ALGO_WIDTH or img.height() != self.cfg.ALGO_HEIGHT:
                print(
                    "[camera.self_test] size mismatch %dx%d vs %dx%d"
                    % (
                        img.width(),
                        img.height(),
                        self.cfg.ALGO_WIDTH,
                        self.cfg.ALGO_HEIGHT,
                    )
                )
                ok = False
                break
            img = None
        return ok
