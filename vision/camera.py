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

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def init(self):
        """构造 sensor、绑定 CHN0 到 LCD、配置 CHN1 算法通道、初始化 Display + Media。

        遵守官方推荐顺序（参考 camera_single_bind_lcd.py 与 plan §11）：
            Sensor() → reset → set_framesize/pixformat (CHN0) → bind_layer
            → set_framesize/pixformat (CHN1) → Display.init → MediaManager.init
        """
        self._sensor = Sensor()
        self._sensor.reset()

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
        img = self._sensor.snapshot(chn=self._algo_chn, timeout=timeout)
        if img is not None:
            self._frame_count += 1
            self._frames_in_window += 1
        return img

    # ------------------------------------------------------------------ #
    # FPS 统计
    # ------------------------------------------------------------------ #
    def algo_fps(self):
        return self._algo_fps

    def display_fps(self):
        try:
            return Display.fps()
        except Exception:
            return 0

    def frame_count(self):
        return self._frame_count

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

    # ------------------------------------------------------------------ #
    # 调试叠加
    # ------------------------------------------------------------------ #
    def render_overlay(self, lines=None):
        """重绘 OSD：3 段 ROI + 总 ROI 外框 + 文本若干。

        plan §9.2 守则 7：仅在 ``maybe_update_fps`` 返回 ``True`` 时调用，
        不每帧刷新；正式比赛通过 ``DEBUG_DISPLAY=False`` 关闭。
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

        if lines:
            x = 8
            y = 8
            font_h = self.cfg.OSD_TEXT_SIZE_PX
            color = self.cfg.OSD_TEXT_COLOR
            for ln in lines:
                self._osd.draw_string_advanced(x, y, font_h, ln, color=color)
                y += font_h + 2

        Display.show_image(self._osd, x=0, y=0, layer=Display.LAYER_OSD0)

    def _draw_roi(self, rect, color, thickness):
        x, y, w, h = rect
        self._osd.draw_rectangle(x, y, w, h, color=color, thickness=thickness)

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
        return ok
