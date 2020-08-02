#!/usr/bin/env python
import rospy
import numpy as np
from mps_shape_completion_visualization.quick_publish import publish_voxelgrid, publish_object_transform

if __name__ == "__main__":
    """
    This is an example showing how to use the publisher.
    Open an rviz file and add the "VoxelGrid" message on the right topic and view the results
    """
    rospy.init_node("quick_voxelgrid_publisher_demo")
    publish_object_transform()
    vg = np.random.random((64, 64, 64)) < 0.01
    publish_voxelgrid(vg, "gt_voxel_grid")
