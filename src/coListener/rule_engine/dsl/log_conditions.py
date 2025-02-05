from .condition import Condition
from .base_conditions import and_, msg, or_, msgtype
from .utils.enum import Enum

# TODO: Add Tests
# TODO: Add other fields that are common for logs

_is_foxglove = or_(msgtype == "foxglove_msgs/Log", msgtype == "foxglove.Log")
_is_ros = msgtype == "rosgraph_msgs/Log"

LogLevel = Enum("LogLevel", ["UNKNOWN", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"])

log = or_(
    and_(_is_ros, msg.msg),
    and_(_is_foxglove, msg.message),
    # Default case
    "",
)

log_level = or_(
    and_(_is_ros, Condition.map(msg, lambda m: ros_log_level(m.level))),
    and_(_is_foxglove, Condition.map(msg, lambda m: foxglove_log_level(m.level))),
    # Default case
    LogLevel.UNKNOWN,
)


def ros_log_level(num):
    if num == 1:
        return LogLevel.DEBUG
    elif num == 2:
        return LogLevel.INFO
    elif num == 4:
        return LogLevel.WARN
    elif num == 8:
        return LogLevel.ERR
    elif num == 16:
        return LogLevel.FATAL
    else:
        return LogLevel.UNKNOWN


def foxglove_log_level(num):
    if num == 1:
        return LogLevel.DEBUG
    elif num == 2:
        return LogLevel.INFO
    elif num == 3:
        return LogLevel.WARN
    elif num == 4:
        return LogLevel.ERROR
    elif num == 5:
        return LogLevel.FATAL
    else:
        return LogLevel.UNKNOWN


__all__ = [
    "log",
    "log_level",
    "LogLevel",
]
