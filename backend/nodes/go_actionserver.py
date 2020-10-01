#! /usr/bin/env python3
"""ROS Node for the Actionlib Server
    """
import time
from math import sqrt
import rospy

from actionlib import SimpleActionServer
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose

from dynamic_reconfigure.server import Server
from backend.cfg import SpeedConfig
from backend.srv import Pause, PauseResponse


from backend.msg import GoAction, GoResult, GoFeedback

MAX_WAITING_TIME = 6
ANGLE_TOLERANCE = 1
DISTANCE_TOLERANCE = 0.5
PI = 3.1415926535897


class GoToGoalAction():
    """Class implementing ActionLibServer that will connect
    and communicate to /turtle1/cmd_vel and update the movement
    It uses multiple ROS functionalities for
    * An Actionlib server that listens and update about the movement
    * The Core Movement functionality that will translate the turtle
    * Speed Dynamic Reconfigure Functionality to change the speed of the movement
    * Pause Service to enable/disable movement

    Args:
        name: The name of the action
    """
    _feedback = GoFeedback()
    _result = GoResult()

    def __init__(self, name):
        # Initializing Critical ROS Features
        self._action_name = name
        self.rate = rospy.Rate(100)

        # ActionServer Related functionality #AS
        self._as = SimpleActionServer(name,
                                      GoAction,
                                      self.goal_cb,
                                      auto_start=False)
        self._as.register_preempt_callback(self.preempt)
        self._as.start()
        rospy.loginfo(
            "[action_server] Started Movement Node, waiting for update on the /turtle1/goal topic")

        # Movement Python Functionality #MOVE

        self.velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(
            '/turtle1/pose', Pose, self.update_pose)
        self.max_distance = 1000
        self.start_time = rospy.Duration()
        self.goal_pose = Pose2D()
        self.pose = Pose()

        # Toggle Service Related functionality #TOGGL
        self.pause_service = rospy.Service(
            'turtle1/pause', Pause, self.pause)
        self.is_paused = False

        # DynRec Speed Server Related functionality #SPEED
        Server(SpeedConfig, self.update_speed_param)
        self.speed = 100

        rospy.loginfo('[%s]: Started Server', self._action_name)
    # Callbacks

    def update_pose(self, data):
        self.pose = data

    # AS
    # Action Server Related Functionality #AS

    def _publish_feedback(self, _percentage=None):
        """This will be called on each translate
        """
        percentage = abs(
            int(float(self.get_distance() / self.max_distance)*100)-100)
        rospy.loginfo("[%s]: Will send percentage: %d" %
                      (self._action_name, percentage))
        self._feedback.percentage = percentage if _percentage is None else _percentage
        self._as.publish_feedback(self._feedback)

    def preempt(self, result=None):
        """This will be activated whenever we cancel the callback
        """
        rospy.loginfo('[%s]:[action_server] Preempted', self._action_name)
        self._as.set_preempted(result)

    def get_elapsed(self) -> rospy.Duration:
        """This will return the time the server has been executing the robot duration
        Returns:
            The Duration of the process
        """
        return rospy.Duration.from_sec(time.time() - self.start_time)

    def goal_cb(self, goal):
        self.goal_pose = goal
        rospy.loginfo('[%s]:[action_server] Executing, making bot go to Goal (%.1f,%.1f)'
                      % (self._action_name, self.goal_pose.x, self.goal_pose.y))
        self.start_time = time.time()
        while (time.time() - self.start_time) < MAX_WAITING_TIME:
            self.move2goal()
            # if self._as.is_preempt_requested():
            #     self.preempt()

        self._result.time_elapsed = self.get_elapsed()
        if self._feedback.percentage == 100:
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('[%s]:[action_server] Operation took too long: %d:%d' %
                          (self._action_name, self._result.time_elapsed.secs,
                           self._result.time_elapsed.nsecs))
            self.preempt(self._result)

    # MOVEMENT
    # The Core Movement functionality that will translate the turtle
    def rotate(self, angle=90):
        if not self.is_paused:
            vel_msg = Twist()
            angular_speed = (self.speed)*2*PI/360
            relative_angle = angle*2*PI/360
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.z = abs(angular_speed)
            t_0 = rospy.Time.now().to_sec()
            current_angle = 0

            while current_angle < relative_angle:
                rospy.loginfo("[%s]:[move]:[ndeg] rotating %.2f < %.2f | %.1f " %
                              (self._action_name, current_angle, relative_angle, self.pose.theta))
                self.velocity_publisher.publish(vel_msg)
                t_1 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t_1-t_0)
                self.rate.sleep()

            rospy.loginfo("[%s]:[move]:[ndeg] Ended rotation because %.2f > %.2f" %
                          (self._action_name, current_angle, relative_angle))

            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
        else:
            rospy.loginfo(
                "[%s]:[move]:[pause] It will not rotate as it is paused" % (self._action_name))

    def get_distance(self):
        distance = abs(sqrt((self.get_x_distance() ** 2) +
                            (self.get_y_distance() ** 2)))
        return distance

    def get_y_distance(self):
        distance = (self.goal_pose.y - self.pose.y)

        return distance

    def get_x_distance(self):
        distance = (self.goal_pose.x - self.pose.x)
        return distance

    def reset_angle(self):
        if not self.is_paused:
            rospy.loginfo(
                "[%s]:[move]:[0deg] should reset 0 , current is %.2f",
                self._action_name, abs(self.pose.theta))
            while abs(round(self.pose.theta)) > ANGLE_TOLERANCE:
                rospy.loginfo("[%s]:[move]:[0deg] rotating  while (%.2f>%.2f)",
                              self._action_name, abs(self.pose.theta), ANGLE_TOLERANCE)
                self.rotate()
            self.rate.sleep()
        else:
            rospy.loginfo(
                "[%s]:[move]:[pause] It will not rotate as it is paused", self._action_name)

    def translate(self):
        if not self.is_paused:
            velocity = 1.0 * (self.speed/100)
            vel_msg = Twist()
            vel_msg.linear.x = abs(velocity)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            self.velocity_publisher.publish(vel_msg)
            self._publish_feedback()
            self.rate.sleep()
        else:
            rospy.loginfo(
                "[%s]:[move] It will not translate as it is paused", self._action_name)

    def move2goal(self):
        rospy.loginfo(self.pose)
        rospy.loginfo("[%s]:[START] Goal (%d,%d) | current is (%.2f,%.2f) at %.3f deg",
                      self._action_name, self.goal_pose.x, self.goal_pose.y,
                      self.pose.x, self.pose.y, self.pose.theta)
        self._publish_feedback(0)
        self.max_distance = self.get_distance()
        self.reset_angle()  # Step 2

        if self.get_x_distance() < 0:  # Step 3
            rospy.loginfo("[%s]:[move]:[fixedg] Will rotate angle 180d as it is %.2f, <-",
                          self._action_name, self.pose.theta)
            self.rotate(180)
        else:
            rospy.loginfo("[%s]:[move]:[ndeg]:[fixdeg] Will not rotate angle as it is %.2f ->",
                          self._action_name, self.pose.theta)

        while abs(self.get_x_distance()) >= DISTANCE_TOLERANCE:  # Step 4
            rospy.loginfo(
                "[%s]:[move] Move in X axis curr:%.2f,want: %.2f | %.2f",
                self._action_name, self.pose.x, self.goal_pose.x, self.get_x_distance())
            self.translate()

        if self.get_y_distance() > 0:  # Step 6
            rospy.loginfo(
                "[%s]:[move]:[fixdeg] Will rotate angle 90 as it is %d ^",
                self._action_name, self.pose.theta)
            self.rotate(90 if self.pose.theta <= PI else 270)
        else:
            rospy.loginfo("[%s]:[move]:[fixdeg] Will rotate angle 270 (270) as it is %d V",
                          self._action_name, self.pose.theta)
            self.rotate(270 if self.pose.theta <= PI else 90)

        while abs(self.get_y_distance()) >= DISTANCE_TOLERANCE:  # Step 7
            rospy.loginfo(
                "[%s]:[move]: Move in Y axis curr:%.2f,want: %.2f | %.2f",
                self._action_name, self.pose.y, self.goal_pose.y, self.get_y_distance())
            # rospy.loginfo("%.3f>=%.3f" %
            #   (self.get_y_distance(), DISTANCE_TOLERANCE))
            self.translate()

        rospy.loginfo("[%s]:[move]:[END] Arrived at destination with dx :%.2f and dy:%.2f",
                      self._action_name, abs(self.get_x_distance()), abs(self.get_y_distance()))
        # Stopping our robot after the movement is over
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        self._publish_feedback(100)

    # SPEED
    # DynRec SPEED Related Functionality  # SPEED
    def update_speed_param(self, config, _level):
        """
            On the init function the SpeedServer was created and each time
            it is changed it will call
            update_speed_param which will update the self.speed which is
            used on the translate function and rotate
            when preparing the changes to the / turtle1/cmd_vel publisher
        """
        self.speed = config.speed_param
        rospy.loginfo(
            "[dynamic_reconfigure]: Updated speed_param to %f " % (self.speed))
        return config

    # PAUSE
    # Toggle Service Related functionality  # TOGGL
    def pause(self, req):
        """
            On the init function the Pause was created and each time
            it is changed it will call
            pause which will update the self.is_paused value that enables
            or disables functionality on
            translate() and reset_angle()
        """
        is_paused = req.is_paused
        self.is_paused = bool(is_paused)
        rospy.loginfo('[%s]:[pause]: Server is now %s' %
                      (self._action_name, 'Paused'if is_paused else 'Normal'))
        return PauseResponse(not is_paused)


if __name__ == '__main__':
    rospy.init_node('go_to_goal')
    server = GoToGoalAction('go_to_goal')
    rospy.spin()
