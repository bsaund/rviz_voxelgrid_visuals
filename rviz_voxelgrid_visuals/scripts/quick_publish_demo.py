#!/usr/bin/env python
import rospy
import numpy as np
from rviz_voxelgrid_visuals import conversions
from rviz_voxelgrid_visuals_msgs.msg import VoxelgridStamped
import pickle
import rospkg

"""
This is an example showing how to use the voxelgrid publisher
Open Rviz with the included `example_voxelgrid.rviz` file, then run this script
"""


if __name__ == "__main__":

    rospy.init_node("quick_voxelgrid_publisher_demo")
    pub_1 = rospy.Publisher('/voxelgrid_1', VoxelgridStamped, queue_size=1)
    pub_2 = rospy.Publisher('/voxelgrid_2', VoxelgridStamped, queue_size=1)

    i = 1
    while pub_1.get_num_connections() == 0 or pub_2.get_num_connections() == 0:
        i += 1
        if i % 10 == 0:
            rospy.loginfo("Waiting for publishers to connect")
        rospy.sleep(0.1)

    # Display some random voxels in rviz
    vg_1 = np.clip(np.random.random((64, 64, 64)) * 100.0 - 99.0, 0, 1) # Only 1% of voxels have a non-zero value
    pub_1.publish(conversions.vox_to_voxelgrid_stamped(vg_1,
                                                       scale=0.01, # Each voxel is a 1cm cube
                                                       frame_id='world', # In frame "world", same as rviz fixed frame
                                                       origin=[-32 * 0.01]*3)) # With origin centering the voxelgrid

    # Load a voxelgrid of a mug and display in rviz
    mug_path = rospkg.RosPack().get_path('rviz_voxelgrid_visuals') + '/examples/mug_voxelgrid.pkl'
    with open(mug_path) as f:
        vg_2 = pickle.load(f)
    pub_2.publish(conversions.vox_to_voxelgrid_stamped(vg_2, scale=0.02, frame_id='world',
                                                       origin=(-.64, -.64, 0)))


