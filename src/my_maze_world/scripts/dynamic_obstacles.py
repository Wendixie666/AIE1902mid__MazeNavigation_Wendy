#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import math

def linear_pingpong(start, end, period, t):
    # smooth ping-pong using cosine easing (0->1->0)
    tau = (t % period) / period
    s = 0.5 * (1 - math.cos(2 * math.pi * tau))
    x = start[0] + (end[0] - start[0]) * s
    y = start[1] + (end[1] - start[1]) * s
    z = start[2] + (end[2] - start[2]) * s
    return x, y, z

def circular(center, radius, period, t):
    angle = 2 * math.pi * ((t % period) / period)
    x = center[0] + radius * math.cos(angle)
    y = center[1] + radius * math.sin(angle)
    z = center[2]
    return x, y, z

def main():
    rospy.init_node('dynamic_obstacles_mover')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # small pause to ensure Gazebo service is fully ready and avoid transient
    # 'Connection refused' errors when calling the service immediately
    rospy.sleep(0.5)

    # run obstacles at a slower, synchronized rate
    rate_hz = rospy.get_param('~rate', 8.0)
    rate = rospy.Rate(rate_hz)

    # slow periods to keep obstacles from moving too fast
    period1 = rospy.get_param('~period1', 12.0)  # was 6, slowed to 12s
    period2 = rospy.get_param('~period2', 16.0)  # was 8, slowed to 16s
    period3 = rospy.get_param('~period3', 20.0)  # was 10, slowed to 20s

    start_time = rospy.Time.now().to_sec()

    # trajectory definitions (match names in maze22.world)
    obs1_start = (-8.0, -6.0, 0.4)
    obs1_end   = (-4.0, -6.0, 0.4)

    obs2_start = (5.0, -8.0, 0.4)
    obs2_end   = (5.0, -4.0, 0.4)

    obs3_center = (0.0, 2.0, 0.4)
    obs3_radius = 1.5

    rospy.loginfo('dynamic_obstacles_mover started (rate: %.1f Hz)', rate_hz)

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec() - start_time

        # obstacle 1 (linear ping-pong)
        p1 = linear_pingpong(obs1_start, obs1_end, period1, now)
        ms1 = ModelState()
        ms1.model_name = 'dynamic_obstacle_1'
        ms1.pose = Pose()
        ms1.pose.position.x = p1[0]
        ms1.pose.position.y = p1[1]
        # lock Z to maze ground plane
        ms1.pose.position.z = 0.0
        ms1.twist = Twist()
        ms1.reference_frame = 'world'
        try:
            set_state(ms1)
        except rospy.ServiceException as e:
            rospy.logwarn('set_model_state failed for obs1: %s', e)

        # obstacle 2 (linear ping-pong)
        p2 = linear_pingpong(obs2_start, obs2_end, period2, now)
        ms2 = ModelState()
        ms2.model_name = 'dynamic_obstacle_2'
        ms2.pose = Pose()
        ms2.pose.position.x = p2[0]
        ms2.pose.position.y = p2[1]
        # lock Z to maze ground plane
        ms2.pose.position.z = 0.0
        ms2.twist = Twist()
        ms2.reference_frame = 'world'
        try:
            set_state(ms2)
        except rospy.ServiceException as e:
            rospy.logwarn('set_model_state failed for obs2: %s', e)

        # obstacle 3 (circular)
        p3 = circular(obs3_center, obs3_radius, period3, now)
        ms3 = ModelState()
        ms3.model_name = 'dynamic_obstacle_3'
        ms3.pose = Pose()
        ms3.pose.position.x = p3[0]
        ms3.pose.position.y = p3[1]
        # lock Z to maze ground plane
        ms3.pose.position.z = 0.0
        ms3.twist = Twist()
        ms3.reference_frame = 'world'
        try:
            set_state(ms3)
        except rospy.ServiceException as e:
            rospy.logwarn('set_model_state failed for obs3: %s', e)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
