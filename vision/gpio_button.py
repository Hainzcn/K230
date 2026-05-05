"""GPIO 按键输入封装。"""

import time


class GpioButton:
    """轮询式外接按键，返回去抖后的单次按下事件。"""

    def __init__(self, cfg, name="button"):
        self.cfg = cfg
        self.name = name
        self.enabled = False
        self.pin = None

        self.io = int(getattr(cfg, "DEBUG_BINARY_BUTTON_IO", -1))
        self.active_low = bool(getattr(cfg, "DEBUG_BINARY_BUTTON_ACTIVE_LOW", True))
        self.debounce_ms = max(
            5, int(getattr(cfg, "DEBUG_BINARY_BUTTON_DEBOUNCE_MS", 80))
        )

        self._last_raw_pressed = False
        self._stable_pressed = False
        self._reported_pressed = False
        self._last_change_ms = 0

        if not getattr(cfg, "DEBUG_BINARY_BUTTON_ENABLE", False):
            return
        self._init_pin()

    def _init_pin(self):
        try:
            from machine import FPIOA, Pin
        except Exception as e:
            print("[button] machine GPIO unavailable:", e)
            return

        if self.io < 0 or self.io > 63:
            print("[button] invalid GPIO io:", self.io)
            return

        try:
            fpioa = FPIOA()
            gpio_func = getattr(FPIOA, "GPIO%d" % self.io)
            if self.active_low:
                fpioa.set_function(self.io, gpio_func, ie=1, pu=1, pd=0)
                pull = Pin.PULL_UP
            else:
                fpioa.set_function(self.io, gpio_func, ie=1, pu=0, pd=1)
                pull = Pin.PULL_DOWN
            self.pin = Pin(self.io, Pin.IN, pull=pull)

            pressed = self._read_pressed()
            self._last_raw_pressed = pressed
            self._stable_pressed = pressed
            self._reported_pressed = pressed
            self._last_change_ms = time.ticks_ms()
            self.enabled = True
            print(
                "[button] %s on IO_%d active_%s debounce=%dms"
                % (
                    self.name,
                    self.io,
                    "low" if self.active_low else "high",
                    self.debounce_ms,
                )
            )
        except Exception as e:
            self.pin = None
            self.enabled = False
            print("[button] init failed:", e)

    def _read_pressed(self):
        level = self.pin.value()
        if self.active_low:
            return level == 0
        return level != 0

    def poll(self, now_ms=None):
        """稳定按下一次返回 True；松开前不会重复触发。"""
        if not self.enabled or self.pin is None:
            return False

        now = now_ms if now_ms is not None else time.ticks_ms()
        try:
            raw_pressed = self._read_pressed()
        except Exception as e:
            print("[button] read failed:", e)
            self.enabled = False
            return False

        if raw_pressed != self._last_raw_pressed:
            self._last_raw_pressed = raw_pressed
            self._last_change_ms = now
            return False

        if time.ticks_diff(now, self._last_change_ms) < self.debounce_ms:
            return False

        if raw_pressed != self._stable_pressed:
            self._stable_pressed = raw_pressed
            if self._stable_pressed:
                if not self._reported_pressed:
                    self._reported_pressed = True
                    return True
            else:
                self._reported_pressed = False

        return False
