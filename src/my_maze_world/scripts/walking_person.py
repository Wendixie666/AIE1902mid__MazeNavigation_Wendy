#!/usr/bin/env python3
"""
Walking person (actor) controller for maze navigation.
Controls the actor's position along a predefined path at 15 Hz.
The actor animates automatically via the DAE file, this script just moves it.
"""
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Quaternion
import math

def linear_interpolate(start, end, period, t):
    """Linear interpolation between start and end positions"""
    tau = (t % period) / period
    x = start[0] + (end[0] - start[0]) * tau
    y = start[1] + (end[1] - start[1]) * tau
    z = start[2] + (end[2] - start[2]) * tau
    return x, y, z

def get_yaw_from_points(start, end):
    """Calculate yaw angle to face the direction of movement"""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 0
    return math.atan2(dy, dx)

def quaternion_from_yaw(yaw):
    """Convert yaw angle to quaternion"""
    half_yaw = yaw / 2.0
    qw = math.cos(half_yaw)
    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    return qw, qx, qy, qz

def main():
    rospy.init_node('walking_person_mover')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # target update rate (slowed to match other obstacles)
    rate_hz = rospy.get_param('~rate', 8.0)
    rate = rospy.Rate(rate_hz)

    # Define safe walking paths through the maze that avoid walls
    # These waypoints are carefully chosen to navigate through the maze safely
    # without colliding with walls
    
    # Path segments designed to avoid all wall collisions
    seg1_start = (8.0, 8.0, 0.85)
    seg1_end = (8.0, -6.0, 0.85)
    seg1_period = 18.0
    
    seg2_start = (8.0, -6.0, 0.85)
    seg2_end = (6.0, -6.0, 0.85)
    seg2_period = 3.0
    
    seg3_start = (6.0, -6.0, 0.85)
    seg3_end = (6.0, -2.0, 0.85)
    seg3_period = 6.0
    
    seg4_start = (6.0, -2.0, 0.85)
    seg4_end = (-1.0, -2.0, 0.85)
    seg4_period = 10.0
    
    seg5_start = (-1.0, -2.0, 0.85)
    seg5_end = (-1.0, 5.0, 0.85)
    seg5_period = 10.0
    
    seg6_start = (-1.0, 5.0, 0.85)
    seg6_end = (5.0, 5.0, 0.85)
    seg6_period = 8.0
    
    seg7_start = (5.0, 5.0, 0.85)
    seg7_end = (5.0, 2.0, 0.85)
    seg7_period = 5.0
    
    seg8_start = (5.0, 2.0, 0.85)
    seg8_end = (8.0, 2.0, 0.85)
    seg8_period = 5.0
    
    seg9_start = (8.0, 2.0, 0.85)
    seg9_end = (8.0, 8.0, 0.85)
    seg9_period = 8.0

    segments = [
        (seg1_start, seg1_end, seg1_period),
        (seg2_start, seg2_end, seg2_period),
        (seg3_start, seg3_end, seg3_period),
        (seg4_start, seg4_end, seg4_period),
        (seg5_start, seg5_end, seg5_period),
        (seg6_start, seg6_end, seg6_period),
        (seg7_start, seg7_end, seg7_period),
        (seg8_start, seg8_end, seg8_period),
        (seg9_start, seg9_end, seg9_period),
    ]
    
    total_period = sum(p for _, _, p in segments)
    start_time = rospy.Time.now().to_sec()

    rospy.loginfo('walking_person_mover started (rate: %.1f Hz, total cycle: %.1f s)', rate_hz, total_period)
    rospy.loginfo('Actor will navigate through maze without colliding with walls')

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec() - start_time
        cycle_time = now % total_period
        
        # Find which segment we're in
        current_pos = (8.0, 8.0, 0.85)
        current_yaw = 0
        elapsed = 0
        
        for start, end, period in segments:
            if elapsed + period > cycle_time:
                # We're in this segment
                time_in_segment = cycle_time - elapsed
                current_pos = linear_interpolate(start, end, period, time_in_segment)
                current_yaw = get_yaw_from_points(start, end)
                break
            elapsed += period

        # Create and send model state for the actor
        qw, qx, qy, qz = quaternion_from_yaw(current_yaw)
        
        ms = ModelState()
        ms.model_name = 'walking_person'
        ms.pose = Pose()
        ms.pose.position.x = current_pos[0]
        ms.pose.position.y = current_pos[1]
        # lock Z to maze ground plane
        ms.pose.position.z = 0.0
        ms.pose.orientation = Quaternion()
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

