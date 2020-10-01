#!/usr/bin/env python3

import dynamic_reconfigure.client
import unittest
import rospy

from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
from std_srvs.srv import Empty

from backend.srv import Pause, PauseResponse, PauseRequest
PKG = 'backend'
TEST_NAME = 'move_integration_test'
TIMEOUT = rospy.Duration(5)
DISTANCE_TOLERANCE = 1.1


def get_pose(x_value: int, y_value: int) -> Pose2D:
    pose = Pose2D()  # Step 1
    pose.x = x_value  # input('Set your x goal:')
    pose.y = y_value   # input('Set your y goal:')
    pose.theta = 0
    return pose


def reset():
    reset_ = rospy.ServiceProxy('reset', Empty)
    reset_()
    rospy.wait_for_service('reset')


class MoveIntegrationTest(unittest.TestCase):
    def __init__(self, *args):
        super(MoveIntegrationTest, self).__init__(*args)

    def _update_pose(self, pose: Pose2D):
        self.position = pose

    def setUp(self):
        rospy.init_node(TEST_NAME, log_level=rospy.DEBUG)
        self.position = Pose2D()
        self._goalpub = rospy.Publisher("/turtle1/goal",
                                        Pose2D, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(
            '/turtle1/pose', Pose, self._update_pose)

    def assert_close_enough(self, goal: Pose2D):
        dx_ = abs(round((self.position.x - goal.x), 4))
        dy_ = abs(round((self.position.x - goal.y), 4))
        self.assertLessEqual(dx_, DISTANCE_TOLERANCE)
        self.assertLessEqual(dy_, DISTANCE_TOLERANCE)

    def assert_moves_to_goal(self, x_value: int, y_value: int, timeout=TIMEOUT):
        pose = get_pose(x_value, y_value)
        rospy.sleep(1)
        self._goalpub.publish(pose)
        rospy.sleep(timeout)
        self.assert_close_enough(pose)
        rospy.sleep(10)

    # def test_speed_changes(self):
    #     start_time = time.time()

    #     # TODO: Make speed dynrec param actually change the speed of the algorithm
    #     assertLess(duration,fasterDuration, "Changing the speed did not make it go faster")

    def test_pause_service_false(self):
        req = PauseRequest()
        req.is_paused = False
        toggle = rospy.ServiceProxy(
            '/turtle1/pause', Pause)
        res = toggle(req)
        rospy.wait_for_service('/turtle1/pause')
        self.assertFalse(res.is_paused)

    def test_pause_service_true(self):
        req = PauseRequest()
        req.is_paused = True
        toggle = rospy.ServiceProxy(
            '/turtle1/pause', Pause)
        res = toggle(req)
        rospy.wait_for_service('/turtle1/pause')
        self.assertTrue(res.is_paused)
        pose = get_pose(10, 10)
        self._goalpub.publish(pose)
        self.assertLogs("It will not rotate as it is paused")

    # def test_move_to_corner_ne(self):
    #     reset()
    #     self.assert_moves_to_goal(9, 9)

    # def test_move_to_corner_nw(self):
    #     reset()
    #     self.assert_moves_to_goal(1, 10)

    # def test_move_to_corner_sw(self):
    #     reset()
    #     self.assert_moves_to_goal(1, 1, 100)

    # def test_move_to_corner_se(self):
    #     reset()
    #     self.assert_moves_to_goal(10, 1, 100)

    # def test_sequential_move(self):
    #     reset()
    #     self.assert_moves_to_goal(10, 1, 100)
    #     self.assert_moves_to_goal(1, 1, 100)
    #     self.assert_moves_to_goal(10, 10, 100)
    #     self.assert_moves_to_goal(10, 1, 100)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, TEST_NAME, MoveIntegrationTest)
