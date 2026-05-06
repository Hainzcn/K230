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
from vision.debug_overlay import DebugOverlay
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
        self._debug_overlay = DebugOverlay(self.cfg)
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
        self._debug_overlay.set_osd(self._osd)

        # ---- 调试 overlay：计算坐标（不再预分配 blit/mask 中间图）---- #
        self._debug_overlay.setup()

        return self

    def start(self):
        """启动 sensor 数据流。MediaManager.init 之后调用（参见 Sensor 文档）。"""
        if self._sensor is None:
            raise RuntimeError("Camera.init() must be called before start()")
        self._sensor.run()
        self._sensor_running = True
        self._debug_overlay.reset_fps_window(time.ticks_ms())
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
            self._debug_overlay.on_frame()
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
        return self._debug_overlay.algo_fps()

    def algo_period_ms(self):
        """算法侧平均帧周期（ms）= 1000 / algo_fps，``algo_fps=0`` 时返回 0。"""
        return self._debug_overlay.algo_period_ms()

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
        return self._debug_overlay.maybe_update_fps(now_ms)

    def maybe_update_binary(self, now_ms=None):
        """二值 overlay 是否到了刷新时刻（独立于 FPS 文字行）。

        ``OSD_BINARY_REFRESH_MS=0`` 时永远返回 True（每帧刷新）；正数时按
        该 ms 间隔节流。返回 True 表示主循环应当调用 :meth:`render_overlay`
        把最新 ``detection`` 画上 OSD。
        """
        return self._debug_overlay.maybe_update_binary(now_ms)

    def binary_overlay_enabled(self):
        """返回主画面 ROI 红色二值 overlay 当前运行期状态。"""
        return self._debug_overlay.binary_overlay_enabled()

    def set_binary_overlay_enabled(self, enabled):
        """运行期切换主画面 ROI 红色二值 overlay。"""
        return self._debug_overlay.set_binary_overlay_enabled(enabled)

    # ------------------------------------------------------------------ #
    # 调试叠加
    # ------------------------------------------------------------------ #
    def render_overlay(self, lines=None, detection=None, path=None):
        """重绘 OSD：3 段 ROI + 总 ROI 外框 + 若干行文本 + (可选) 检测 / 路径可视化。

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
        :param path: 可选 :class:`vision.debug_overlay.PathOverlayInfo`。
            阶段 C 起非空时画 IPM 后的 5 点折线、圆心反投点 / ROI 边缘箭头、
            近带切线箭头。``path.calib_mode == "none"`` 时跳过几何，仅靠
            调用方在 ``lines`` 里追加红色 ``NO CALIB`` 文本提醒。
        """
        if not self.cfg.DEBUG_DISPLAY or self._osd is None:
            return

        if self._debug_overlay.draw(lines, detection, path):
            Display.show_image(self._osd, x=0, y=0, layer=Display.LAYER_OSD0)

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
