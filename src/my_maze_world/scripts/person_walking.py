#!/usr/bin/env python3
"""
Slow controller for `person_walking` actor: updates position at low rate to avoid walls.
"""
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Quaternion
import math

def linear_interpolate(start, end, period, t):
    tau = max(0.0, min(1.0, t / period))
    x = start[0] + (end[0] - start[0]) * tau
    y = start[1] + (end[1] - start[1]) * tau
    z = start[2] + (end[2] - start[2]) * tau
    return x, y, z

def get_yaw_from_points(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 0.0
    return math.atan2(dy, dx)

def quaternion_from_yaw(yaw):
    half_yaw = yaw / 2.0
    qw = math.cos(half_yaw)
    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    return qw, qx, qy, qz

def main():
    rospy.init_node('person_walking_mover')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # set slow actor update to 8 Hz to match other obstacles
    rate_hz = rospy.get_param('~rate', 8.0)
    rate = rospy.Rate(rate_hz)

    # safe, slow path avoiding walls (kept well within free corridors)
    segs = [
        ((-1.5, -6.5, 0.85), (-1.5, -2.5, 0.85), 20.0),
        ((-1.5, -2.5, 0.85), (2.5, -2.5, 0.85), 12.0),
        ((2.5, -2.5, 0.85), (2.5, 2.5, 0.85), 14.0),
        ((2.5, 2.5, 0.85), (-1.5, 2.5, 0.85), 12.0),
        ((-1.5, 2.5, 0.85), (-1.5, -6.5, 0.85), 18.0),
    ]

    total = sum(p for _,_,p in segs)
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo('person_walking_mover started at %.1f Hz', rate_hz)

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        ct = t % total
        elapsed = 0.0
        pos = segs[0][0]
        yaw = 0.0
        for s,e,p in segs:
            if elapsed + p >= ct:
                tt = ct - elapsed
                pos = linear_interpolate(s, e, p, tt)
                yaw = get_yaw_from_points(s, e)
                break
            elapsed += p

        qw,qx,qy,qz = quaternion_from_yaw(yaw)
        ms = ModelState()
        ms.model_name = 'person_walking'
        ms.pose = Pose()
        ms.pose.position.x = pos[0]
        ms.pose.position.y = pos[1]
        # lock Z to maze ground plane
        ms.pose.position.z = 0.0
        ms.pose.orientation.w = qw
        ms.pose.orientation.x = qx
        ms.pose.orientation.y = qy
        ms.pose.orientation.z = qz
        ms.twist = Twist()
        ms.reference_frame = 'world'
        try:
            set_state(ms)
        except rospy.ServiceException as e:
            rospy.logwarn('set_model_state failed: %s', e)
        rate.sleep()

if __name__ == '__main__':
    main()
