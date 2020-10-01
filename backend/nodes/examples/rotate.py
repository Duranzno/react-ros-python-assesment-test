#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897
SPEED = 300


def rotate90(callback):
    # msg = Twist()
    # msg.linear.x = 0
    # msg.linear.y = 0
    # msg.linear.z = 0
    # msg.angular.x = 0
    # msg.angular.y = 0
    # msg.angular.z = PI / 4
    rotate(5, callback)


def rotate(angle, callback):
    vel_msg = Twist()
    # Converting from angles to radians
    angular_speed = SPEED*2*PI/360
    relative_angle = angle*2*PI/360

    # We wont use linear components

    vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        callback(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.angular.z = 0
    callback(vel_msg)


def rotate_script():

    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)
    # angle = input("Type your distance (degrees):")
    rospy.sleep(1)
    rotate90(velocity_publisher.publish)
    # msg = Twist()
    # msg.linear.x = 0
    # msg.linear.y = 0
    # msg.linear.z = 0
    # msg.angular.x = 0
    # msg.angular.y = 0
    # msg.angular.z = 3.1415926535897
    # rospy.loginfo(msg)
    # while not rospy.is_shutdown():

    # velocity_publisher.publish(msg)
    # rospy.shutdown()
    # rotate90(velocity_publisher.publish)
    # rospy.spin()


if __name__ == '__main__':
    try:
        # Testing our function
        rotate_script()
    except rospy.ROSInterruptException:
        pass
