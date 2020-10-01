#!/usr/bin/env python3

import unittest
import actionlib

import rospy
from geometry_msgs.msg import Pose2D
from backend.msg import GoAction, GoFeedback
from std_srvs.srv import Empty


PKG = 'backend'
TEST_NAME = 'publish_goal_integration_test'
TIMEOUT = rospy.Duration(100)


def get_pose(x_value, y_value):
    pose = Pose2D()  # Step 1
    pose.x = x_value  # input('Set your x goal:')
    pose.y = y_value   # input('Set your y goal:')
    pose.theta = 0
    return pose


def reset():
    rospy.ServiceProxy("/reset", Empty).call()
    rospy.Rate(10).sleep()


class PublishGoalIntegrationTest(unittest.TestCase):

    def __init__(self, *args):
        super(PublishGoalIntegrationTest, self).__init__(*args)

    def setUp(self):
        rospy.init_node(TEST_NAME, log_level=rospy.DEBUG)
        rospy.loginfo("RESETED")

    def assert_goal(self, terminal_state, _result):
        # rospy.loginfo("[:%s] result " % (TEST_NAME))
        # rospy.loginfo(_result)
        # rospy.loginfo("[:%s] terminal_state " % (TEST_NAME))
        self.assertEqual(terminal_state, 3, "State is not suceeded")
        rospy.loginfo(terminal_state)

    def assert_feedback(self, feedback: GoFeedback):
        rospy.loginfo("[:%s] feedback " % (TEST_NAME))
        rospy.loginfo(feedback)
        self.assertGreaterEqual(feedback.percentage, 0)
        self.assertLessEqual(feedback.percentage, 100)

    def turtle_bot_is_moved(self, goal):
        client = actionlib.SimpleActionClient('go_to_goal', GoAction)

        server_connected = client.wait_for_server(TIMEOUT)
        self.assertTrue(server_connected, "Server did not connect")

        client.send_goal(goal, done_cb=self.assert_goal,
                         feedback_cb=self.assert_feedback)

        result_done = client.wait_for_result(TIMEOUT)
        rospy.loginfo("RESULT")
        rospy.loginfo(result_done)
        self.assertTrue(result_done, "Server did not finished with done")

    def test_turtle_bot_is_moved_tl(self):
        reset()
        self.turtle_bot_is_moved(get_pose(1,    10))

    def test_turtle_bot_is_moved_dr(self):
        reset()
        self.turtle_bot_is_moved(get_pose(10, 1))

    def test_turtle_bot_is_moved_dl(self):
        reset()
        self.turtle_bot_is_moved(get_pose(1, 1))

    def test_turtle_bot_is_moved_tr(self):
        reset()
        self.turtle_bot_is_moved(get_pose(10, 10))

    # def test_turtle_is_moved_sequentially(self):
    #     rospy.loginfo("turtle_bot_is_moved_tl")
    #     rospy.Rate(10).sleep()
    #     self.turtle_bot_is_moved(get_pose(1,    10))

    #     rospy.loginfo("turtle_bot_is_moved_dr")
    #     rospy.Rate(10).sleep()
    #     self.turtle_bot_is_moved(get_pose(10, 1))

    #     rospy.loginfo("turtle_bot_is_moved_dl")
    #     rospy.Rate(10).sleep()
    #     self.turtle_bot_is_moved(get_pose(1, 1))

    #     rospy.loginfo("turtle_bot_is_moved_tr")
    #     rospy.Rate(10).sleep()
    #     self.turtle_bot_is_moved(get_pose(10, 10))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, TEST_NAME, PublishGoalIntegrationTest)
