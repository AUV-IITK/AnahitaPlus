import rospy
from uuv_control_msgs.srv import *
import numpy as np
import uuv_waypoints
from uuv_control_msgs.msg import Waypoint as WaypointMsg 
import time
from std_msgs.msg import Time, String
from geometry_msgs.msg import Point

if __name__ == '__main__':

    rospy.init_node('waypoint_server')

    wp1 = uuv_waypoints.Waypoint(1.5, 1.5, -2.5, 0.15, inertial_frame_id='world')
    wp2 = uuv_waypoints.Waypoint(3, 0, -2.5, 0.15, inertial_frame_id='world')
    wp3 = uuv_waypoints.Waypoint(1.5, -1.5, -2.5, 0.15, inertial_frame_id='world')
    wp4 = uuv_waypoints.Waypoint(0, -0.5, -2.5, 0.15, inertial_frame_id='world')

    wp1_msg = WaypointMsg()
    wp1_msg = wp1.to_message()
    wp2_msg = WaypointMsg()
    wp2_msg = wp2.to_message()
    wp3_msg = WaypointMsg()
    wp3_msg = wp3.to_message()
    wp4_msg = WaypointMsg()
    wp4_msg = wp4.to_message()

    print 'waiting for circular server'
    rospy.wait_for_service('anahita/start_circular_trajectory')

    wp_set = [wp1_msg, wp2_msg, wp3_msg, wp4_msg]

    try:
        init_waypoint_set = rospy.ServiceProxy('anahita/start_waypoint_list', InitWaypointSet)
        init_circular_trajectory = rospy.ServiceProxy('anahita/start_circular_trajectory', InitCircularTrajectory)
        trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
        
        start_time = Time()
        start_time.data.secs = rospy.get_rostime().to_sec()
        start_time.data.nsecs = rospy.get_rostime().to_nsec()
        center = Point()
        center.x = 2
        center.y = 0
        center.z = -2
        
        interpolator = String()
        interpolator.data = 'cubic_interpolator'
        print 'adding waypoints....'
        resp = init_circular_trajectory(start_time = start_time, 
                                        start_now = True,
                                        radius=2,
                                        center=center,
                                        is_clockwise=True,
                                        angle_offset=0.5,
                                        n_points=10,
                                        max_forward_speed = 1,
                                        heading_offset = 0.5,
                                        duration=200)

        # resp = init_waypoint_set(start_time=start_time,
        #                         start_now=True,
        #                         waypoints=wp_set,
        #                         max_forward_speed=0.1,
        #                         heading_offset=0.5,
        #                         interpolator=interpolator)

        if resp.success:
            print "waypoints successfully added"
        else:
            print "waypoints addition failed"

        trajectory_complete_resp = trajectory_complete(time_out=200)

        if trajectory_complete_resp.success:
            print 'trajectory completed successfully'
        else:
            print 'failed at traversing the path'
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e