#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from backend.cfg import SpeedConfig


def callback(config, level):
    rospy.loginfo(config)
    rospy.loginfo(level)
    return config


if __name__ == "__main__":
    rospy.init_node("backend", anonymous=False)
    srv = Server(SpeedConfig, callback)
    rospy.spin()
