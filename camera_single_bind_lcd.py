# Camera Example
import time, os, sys, image

from media.sensor import *
from media.display import *
from media.media import *

sensor = None
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
FPS_OVERLAY_WIDTH = 160
FPS_OVERLAY_HEIGHT = 48
FPS_OVERLAY_MARGIN = 8

try:
    print("camera_test")

    # construct a Sensor object with default configure
    sensor = Sensor()
    # sensor reset
    sensor.reset()
    # set hmirror
    # sensor.set_hmirror(False)
    # sensor vflip
    # sensor.set_vflip(False)

    # set chn0 output size, 800x480
    sensor.set_framesize(width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT)
    # set chn0 output format
    sensor.set_pixformat(Sensor.YUV420SP)
    # bind sensor chn0 to display layer video 1
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer = Display.LAYER_VIDEO1)

    # use lcd as display output
    Display.init(Display.ST7701, width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT, to_ide = True)
    # init media manager
    MediaManager.init()
    fps_overlay = image.Image(FPS_OVERLAY_WIDTH, FPS_OVERLAY_HEIGHT, image.ARGB8888)
    fps_overlay_x = DISPLAY_WIDTH - FPS_OVERLAY_WIDTH - FPS_OVERLAY_MARGIN
    fps_overlay_y = FPS_OVERLAY_MARGIN
    last_fps_update = time.ticks_ms() - 1000
    # sensor start run
    sensor.run()

    while True:
        os.exitpoint()
        now = time.ticks_ms()
        if time.ticks_diff(now, last_fps_update) >= 1000:
            last_fps_update = now
            fps_overlay.clear()
            fps_overlay.draw_string_advanced(0, 0, 32, "FPS:%d" % Display.fps(), color=(255, 255, 255))
            Display.show_image(fps_overlay, x=fps_overlay_x, y=fps_overlay_y, layer=Display.LAYER_OSD0)
        time.sleep_ms(10)
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    # sensor stop run
    if isinstance(sensor, Sensor):
        sensor.stop()
    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()
