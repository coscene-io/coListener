#!/usr/bin/env python
import logging
import os
import logging.handlers

LOGGING_FORMAT = "[%(levelname)s] %(asctime)s [%(filename)s, line: %(lineno)04d]: %(message)s"
fixed_length = 16

logger = logging.getLogger('coListener')
logger.setLevel(logging.INFO)

log_dir = os.path.join(os.getenv('HOME'), '.cache', 'coListener', 'log')
log_file = os.path.join(log_dir, 'coListener.log')

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

file_handler = logging.handlers.TimedRotatingFileHandler(
    log_file,
    when='midnight',
    interval=1,
    backupCount=3,
    encoding='utf-8'
)

class FixedLengthFileNameFormatter(logging.Formatter):
    def __init__(self, fmt=None, datefmt=None):
        super(FixedLengthFileNameFormatter, self).__init__(fmt, datefmt)

    def format(self, record):
        filename = record.filename
        if len(filename) > fixed_length:
            filename = filename[: fixed_length - 3] + "..."
        else:
            filename = filename.rjust(fixed_length)
        record.filename = filename

        if record.levelname == "WARNING":
            record.levelname = " WARN"
        if record.levelname == "INFO":
            record.levelname = " INFO"
        return super(FixedLengthFileNameFormatter, self).format(record)


formatter = FixedLengthFileNameFormatter(fmt=LOGGING_FORMAT)
file_handler.setFormatter(formatter)

logger.addHandler(file_handler)

