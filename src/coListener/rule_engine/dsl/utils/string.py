#!/usr/bin/env python

import re
import sys

def format_string(string):
    if sys.version_info[0] == 2:
        return string.encode('utf-8')
    else:
        return string

