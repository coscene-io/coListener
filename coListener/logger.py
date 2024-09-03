# Copyright 2024 coScene
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import logging
from systemd.journal import JournalHandler


class FixedLengthFileNameFormatter(logging.Formatter):
    def __init__(self, fmt=None, datefmt=None, style="%", fixed_length=16):
        super().__init__(fmt, datefmt, style)
        self.fixed_length = fixed_length

    def format(self, record):
        filename = record.filename
        if len(filename) > self.fixed_length:
            filename = filename[: self.fixed_length - 3] + "..."  # 截断文件名
        else:
            filename = filename.rjust(self.fixed_length)  # 补空格
        record.filename = filename

        if record.levelname == 'WARNING':
            record.levelname = ' WARN'
        if record.levelname == 'INFO':
            record.levelname = ' INFO'
        return super().format(record)


LOGGING_FORMAT = (
    "[%(levelname)s] [%(filename)s, line: %(lineno)04d]: %(message)s"
)
logger = logging.getLogger("coListener")
logger.setLevel(logging.DEBUG)

journal_handler = JournalHandler(SYSLOG_IDENTIFIER='coListener')
journal_handler.setFormatter(FixedLengthFileNameFormatter(fmt=LOGGING_FORMAT))
logger.addHandler(journal_handler)
