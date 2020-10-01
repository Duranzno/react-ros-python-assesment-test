#! /usr/bin/env python3
"""[Test Module to send action to actionlibserver]"""
import rospy

import actionlib
from geometry_msgs.msg import Pose2D
from backend.msg import GoAction


def feedback_cb(feedback):
    """Feedback Callback that will send datae"""
    rospy.loginfo("Progress is: %")
    rospy.loginfo(feedback.percentage)


rospy.init_node('go_to_goal_action_client')
rospy.loginfo('[actionlib-client] Will Start')
client = actionlib.SimpleActionClient('go_to_goal', GoAction)
server_connected = client.wait_for_server()
rospy.loginfo('[actionlib-client] Waited for server')

goal = Pose2D()
goal.x = 10  # input('Set your x goal:')
goal.y = 10  # input('Set your y goal:')
goal.theta = 0
rospy.loginfo('[actionlib-client] Sent Data')
rospy.loginfo(goal)
client.send_goal(goal, feedback_cb=feedback_cb)


client.wait_for_result()
rospy.loginfo('[actionlib-client] Time elapsed:')
rospy.loginfo(client.get_result())
