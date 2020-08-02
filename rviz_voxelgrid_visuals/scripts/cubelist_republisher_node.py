#!/usr/bin/env python


"""
Republishes an occupancy grid from a Float32MultiArray as a cubelist marker viewable in RViz

This listens to "demo_voxel_grid" (which can be remapped) and outputs to "shape_completion_marker"
It looks for "base_frame" and "scale", and "r/g/b/a" parameters, otherwise uses default values
"""

import rospy
import numpy as np
from mps_shape_completion_visualization.conversions import occupancyStamped_to_cubelist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from mps_shape_completion_msgs.msg import OccupancyStamped




def callback(msg):
    color = ColorRGBA()
    color.a = rospy.get_param('~a', 1.0)
    color.r = rospy.get_param('~r', np.random.random())
    color.g = rospy.get_param('~g', np.random.random())
    color.b = rospy.get_param('~b', np.random.random())

    occ_stamped = OccupancyStamped();
    occ_stamped.occupancy = msg
    occ_stamped.header.frame_id = rospy.get_param('~frame_id', "base_frame")
    occ_stamped.header.stamp = rospy.get_rostime()
    occ_stamped.scale = rospy.get_param('~scale', 0.01)
    
    pub.publish(occupancyStamped_to_cubelist(occ_stamped, color))
    

if __name__ == "__main__":
    rospy.init_node("shape_completion_cubelist_republisher")
    pub = rospy.Publisher("shape_completion_marker", Marker, queue_size=100)

    # This topic should be remapped when launching this node
    rospy.Subscriber("demo_voxel_grid", numpy_msg(Float32MultiArray), callback)

    rospy.spin()
