"""调试 OSD 叠加绘制。

把检测几何、二值调试 overlay、OSD 文本与 FPS/二值刷新节流从
``vision.camera`` 中拆出，Camera 只负责提供 OSD buffer 与最终 display 入口。
"""

import time

import config
from vision.interrupts import reraise_if_stop


class DebugOverlay:
    """负责 OSD 上的调试可视化与相关运行期状态。"""

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else config
        self._osd = None

        # 帧统计：Camera 每拿到一帧算法图像后调用 on_frame()。
        self._frames_in_window = 0
        self._algo_fps = 0.0
        self._last_fps_update_ms = 0

        # ROI 在显示坐标下的等比例位置（plan §4.3）。
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
        # 兼容：旧名 "full_dither" → "full_dither_50"。
        if mode == "full_dither":
            mode = "full_dither_50"
        valid_modes = (
            "bands_only", "full_solid",
            "full_dither_50", "full_dither_25", "full_dither_12",
        )
        if mode not in valid_modes:
            print("[debug_overlay] unknown OSD_BINARY_OVERLAY_MODE=%r, fallback to bands_only" % mode)
            mode = "bands_only"
        self._binary_overlay_mode = mode
        # 二值 overlay 刷新间隔（ms）。0 = 每帧；正数 = 节流。
        self._binary_refresh_ms = max(0, int(getattr(self.cfg, "OSD_BINARY_REFRESH_MS", 0)))
        self._last_binary_refresh_ms = 0
        self._binary_dest_x = 0
        self._binary_dest_y = 0
        self._binary_scale_x = 1.0
        self._binary_scale_y = 1.0
        # 调试：右上角原尺寸黑白预览的目标坐标（已知最简通路；无 mask 无缩放）。
        self._binary_preview_x = 0
        self._binary_preview_y = 0
        # 节流：只在前 N 次 OSD 刷新打印诊断日志，避免长时间运行时刷屏。
        self._binary_dbg_remaining = 0

    def set_osd(self, osd):
        self._osd = osd

    def setup(self):
        """完成依赖 OSD/display 几何的二值 overlay 初始化。"""
        if not (self._binary_overlay_enabled or self._binary_preview_enabled):
            return
        try:
            roi_x, roi_y, roi_w, roi_h = self.cfg.ROI_TOTAL_PX
            sx = self.cfg.DISPLAY_WIDTH / float(self.cfg.ALGO_WIDTH)
            sy = self.cfg.DISPLAY_HEIGHT / float(self.cfg.ALGO_HEIGHT)
            self._binary_dest_x = int(roi_x * sx)
            self._binary_dest_y = int(roi_y * sy)
            self._binary_scale_x = sx
            self._binary_scale_y = sy
            # 右上角预览窗坐标：x=DISPLAY_W-roi_w-10, y=10。
            self._binary_preview_x = max(
                0, self.cfg.DISPLAY_WIDTH - roi_w - 10
            )
            self._binary_preview_y = 10
            # 前 10 次 OSD 刷新打印诊断；之后静默。
            self._binary_dbg_remaining = 10
            print(
                "[debug_overlay] binary overlay setup: overlay=%s preview=%s "
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
            print("[debug_overlay] binary overlay setup failed:", e)
            self._binary_overlay_enabled = False
            self._binary_preview_enabled = False

    def on_frame(self):
        self._frames_in_window += 1

    def reset_fps_window(self, now_ms=None):
        self._frames_in_window = 0
        self._last_fps_update_ms = now_ms if now_ms is not None else time.ticks_ms()

    def algo_fps(self):
        return self._algo_fps

    def algo_period_ms(self):
        if self._algo_fps <= 0.0:
            return 0.0
        return 1000.0 / self._algo_fps

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
        """二值 overlay 是否到了刷新时刻（独立于 FPS 文字行）。"""
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
            "[debug_overlay] binary overlay %s"
            % ("ON" if self._binary_overlay_enabled else "OFF")
        )
        return True

    def draw(self, lines=None, detection=None):
        """重绘 OSD：ROI 框、文本，以及可选检测/二值图可视化。"""
        if not self.cfg.DEBUG_DISPLAY or self._osd is None:
            return False

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

        return True

    def _draw_roi(self, rect, color, thickness):
        x, y, w, h = rect
        self._osd.draw_rectangle(x, y, w, h, color=color, thickness=thickness)

    def _draw_detection(self, detection):
        """把 :class:`DetectionResult` 中 5 条带的几何信息画到 OSD。"""
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
            # 扫描带矩形（使用 ROI 全宽，带高 = BAND_HEIGHT_PX）。
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

            # 等效宽度：以 cx 为中心画一段水平线段，长度 = width_px。
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
        """绘制二值图调试 overlay。preview 与 overlay 各自独立。"""
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
                print("[debug_overlay] binary_np → bytes failed:", e)
                return

        if len(binary_bytes) < roi_w * roi_h:
            print(
                "[debug_overlay] binary bytes truncated: %d < %d, abort overlay"
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
                print("[debug_overlay] overlay draw failed:", e)

        if verbose:
            band_area = self.cfg.BAND_HEIGHT_PX * self.cfg.BAND_COUNT * roi_w
            fg_band = int(detection.mass_total) // 255 if band_area else 0
            fg_pct = (100.0 * fg_band / band_area) if band_area else 0.0
            print(
                "[debug_overlay] binary thr=%d band_fg=%d/%d (%.1f%%) "
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
            print("[debug_overlay] preview bg failed:", e)

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
            print("[debug_overlay] preview band scan failed:", e)

        try:
            self._osd.draw_rectangle(
                self._binary_preview_x - 1, self._binary_preview_y - 1,
                roi_w + 2, roi_h + 2,
                color=(255, 255, 0), thickness=1,
            )
        except Exception as e:
            reraise_if_stop(e)
            print("[debug_overlay] preview frame failed:", e)
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
        """主画面 overlay：全 ROI 抖动模式画 1×1 红点。"""
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
            # 50% 棋盘格。
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
            # 25%：偶数行 × 偶数列。
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
        """从 binary_bytes 的第 y 行扫前景连续段并画到 OSD（bands_only / preview 用）。"""
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
        """算法坐标 (px @ 320×240) → 显示坐标 (px @ DISPLAY_WIDTH × DISPLAY_HEIGHT)。"""
        sx = self.cfg.DISPLAY_WIDTH / float(self.cfg.ALGO_WIDTH)
        sy = self.cfg.DISPLAY_HEIGHT / float(self.cfg.ALGO_HEIGHT)
        return int(x * sx), int(y * sy)

    def _roi_algo_to_display(self, roi_algo):
        """把算法坐标系下的 ROI 等比例缩放到显示坐标系。"""
        ax, ay, aw, ah = roi_algo
        sx = self.cfg.DISPLAY_WIDTH / float(self.cfg.ALGO_WIDTH)
        sy = self.cfg.DISPLAY_HEIGHT / float(self.cfg.ALGO_HEIGHT)
        return (int(ax * sx), int(ay * sy), int(aw * sx), int(ah * sy))
