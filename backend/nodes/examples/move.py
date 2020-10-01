#!/usr/bin/env python3
"""ROS Node that listens to the updates in /turtle1/goal topic
    and changes the position of the robot so it can reach that goal
    """
from math import sqrt
import rospy

from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
from std_msgs.msg import Int8
from rotate import rotate90, rotate

from dynamic_reconfigure.server import Server
from backend.cfg import SpeedConfig
from backend.srv import Pause, PauseResponse

ANGLE_TOLERANCE = 0.5
DISTANCE_TOLERANCE = 1


class Movement():
    """ Movement is the central node
    that orchestrates the PauseService and updates the /turtle1/percentage topic
    """

    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('Movement_controller',
                        anonymous=True)
        rospy.loginfo(
            "Started Movement Node, waiting for update on the /turtle1/goal topic")
        # Topic Publishers
        self.velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        self.distance_publisher = rospy.Publisher(
            "/turtle1/percentage", Int8, queue_size=10)
        # Topic Subscribers
        self.pose_subscriber = rospy.Subscriber(
            '/turtle1/pose', Pose, self.update_pose)
        self.pause_service = rospy.Service(
            'turtle1/pause', Pause, self.pause)
        Server(SpeedConfig, self.update_speed_param)
      # Initialization
        self.distance_publisher.publish(0)
        self.is_paused = False
        self.pose = Pose()
        self.goal_pose = Pose2D()
        self.rate = rospy.Rate(10)
        self.max_distance = 1000
        self.speed = 100

    def update_speed_param(self, config, _level):
        rospy.loginfo(config)
        rospy.loginfo(_level)
        rospy.loginfo("Updated speed to %f " % (config.speed_param))
        self.speed = config.speed_param
        return config

    def pause(self, req):
        is_paused = req.is_paused
        self.is_paused = bool(is_paused)
        rospy.loginfo('[pause-service]: called with %s' %
                      ('resume'if is_paused else 'pause'))
        return PauseResponse(is_paused)

    def linear_vel(self, distance, constant=1.5):
        return constant * distance * 1.0

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def update_distance_topic(self):
        percentage = abs(
            int(float(self.get_distance() / self.max_distance)*100)-100)
        self.distance_publisher.publish(percentage)

    def get_distance(self):
        distance = abs(sqrt((self.get_x_distance(self.goal_pose) ** 2) +
                            (self.get_y_distance(self.goal_pose) ** 2)))
        return distance

    def get_y_distance(pose, goal_pose):
        distance = (goal_pose.y - pose.y)
        rospy.loginfo("[dy] Ydistance to goal is =%.2f", distance)

        return distance

    def get_x_distance(pose, goal_pose):
        distance = (goal_pose.x - pose.x)
        rospy.loginfo("[dx] Xdistance to goal is =%.2f", distance)
        return distance

    def reset_angle(self):
        if not self.is_paused:
            rospy.loginfo(
                "[0deg] should reset 0 , current is %.2f", abs(self.pose.theta))
            while abs(round(self.pose.theta)) > ANGLE_TOLERANCE:
                rospy.loginfo("[0deg] rotating  until (%.2f>%.2f)",
                              abs(self.pose.theta), ANGLE_TOLERANCE)
                self.rate.sleep()
                rotate90(self.velocity_publisher.publish)
            self.rate.sleep()
        else:
            rospy.loginfo("It will not rotate as it is paused")

    def translate(self, goal_pose):
        if not self.is_paused:
            velocity = self.linear_vel(goal_pose.x)
            vel_msg = Twist()
            vel_msg.linear.x = abs(velocity)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.update_distance_topic()
            self.rate.sleep()
        else:
            rospy.loginfo("It will not translate as it is paused")

    def move2goal(self, data):
        self.goal_pose.x = data.x
        self.goal_pose.y = data.y
        self.rate.sleep()
        rospy.loginfo(self.pose)
        rospy.loginfo("[START] Goal (%d,%d) | current is (%.2f,%.2f) at %.3f deg",
                      self.goal_pose.x, self.goal_pose.y, self.pose.x, self.pose.y, self.pose.theta)

        self.max_distance = self.get_distance()

        self.reset_angle()  # Step 2

        if self.get_x_distance(self.goal_pose) < 0:  # Step 3
            rospy.loginfo("[fixdeg] Will rotate angle 180d as it is %d, <-",
                          self.pose.theta)
            rotate(180,    self.velocity_publisher.publish)
        else:
            rospy.loginfo("[fixdeg] Will not rotate angle as it is %d ->",
                          self.pose.theta)

        while abs(self.get_x_distance(self.goal_pose)) >= DISTANCE_TOLERANCE:  # Step 4
            rospy.loginfo(
                "[MOV] Will ranslate in X axis  as it is in %d, and we want %d",
                self.pose.x, self.goal_pose.x)
            self.translate(self.goal_pose)

        self.reset_angle()  # step 5
        if self.get_y_distance(self.goal_pose) > 0:  # Step 6
            rospy.loginfo(
                "[fixdeg] Will rotate angle 90 as it is %d ^", self.pose.theta)
            rotate(90,    self.velocity_publisher.publish)
        else:
            rospy.loginfo("[fixdeg] Will rotate angle -90 (270) as it is %d V",
                          self.pose.theta)
            rotate(270,    self.velocity_publisher.publish)

        while abs(self.get_y_distance(self.goal_pose)) >= DISTANCE_TOLERANCE:  # Step 7
            rospy.loginfo(
                "[MOV] Will translate in Y axis  as it is in %d, and we want %d",
                self.pose.y, self.goal_pose.y)
            self.translate(self.goal_pose)

        rospy.loginfo("[END] Arrived at destination with dx :%.2f and dy:%.2f", abs(
            self.get_x_distance(self.goal_pose)), abs(self.get_y_distance(self.goal_pose)))
        # Stopping our robot after the movement is over
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.distance_publisher.publish(100)


if __name__ == '__main__':
    try:
        x = Movement()
        rospy.Subscriber("/turtle1/goal", Pose2D, x.move2goal)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
