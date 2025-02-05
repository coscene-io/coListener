#!/usr/bin/env python

import rospy
from coListener import coListener
from coListener.logger.logger import logger as _log

if __name__ == '__main__':
    try:
        rospy.init_node('coListener', anonymous=True)
        _log.info("-************************* coListener start *************************-")
        coListener.CoListener()
        rospy.spin()
    except Exception as e:
        _log.error(e)
        raise e