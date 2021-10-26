#! /usr/bin/env python

# import ros stuff
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

import math

import random

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 4
desired_position_.y = 0
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 22.5 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None


# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_stop(msg):
    global emergency_stop_
    if msg.data == "True":
       emergency_stop_ = True
       print("Emergency stop triggered")
    else:
       emergency_stop_ = False

def mock_sliding():
    global position_

    mean_ = 2.0
    random_sliding_dist = mean_ + random.random()
    fake_loc_ = Point()
    fake_loc_.x = position_.x + random_sliding_dist
    fake_loc_.y = position_.y
    go_straight_ahead(fake_loc_)

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_

    twist_msg = Twist()
    twist_msg.linear.x = 0.1
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.4 if err_yaw > 0 else -0.4

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        fix_yaw(des_pos)
        change_state(0)


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, emergency_stop_

    rospy.init_node('mock_mission')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    sub_emergency_rospy = rospy.Subscriber("/emergency_stop", String, clbk_stop)
    emergency_stop_ = False

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if emergency_stop_:
            print("Emergency stop triggered stop")
            mock_sliding()
            done()
            change_state(2)
            pass

        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            mock_sliding()
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

if __name__ == '__main__':
    main()
