"""运行期停止信号识别工具。"""


def is_stop_exception(exc):
    """判断异常是否来自 IDE/用户停止请求。"""
    if isinstance(exc, KeyboardInterrupt):
        return True
    try:
        msg = str(exc)
    except Exception:
        msg = ""
    return "IDE interrupt" in msg or "KeyboardInterrupt" in msg


def reraise_if_stop(exc):
    """停止请求不能被调试容错逻辑吞掉。"""
    if is_stop_exception(exc):
        raise exc
