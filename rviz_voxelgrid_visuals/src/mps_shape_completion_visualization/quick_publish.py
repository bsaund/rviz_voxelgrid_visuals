"""
Method for publishing a voxelgrid to rviz. Not fast or efficient, but simple to use
"""
import rospy
from mps_shape_completion_msgs.msg import OccupancyStamped
import mps_shape_completion_visualization.conversions as conversions
import tf2_ros
import tf_conversions
import geometry_msgs.msg


def publish_voxelgrid(vg, topic="voxelgrid", scale=0.01, frame_id="object"):
    """
    @param vg: Assumes a cubic voxel grid (all dimensions are equal)
    @param topic:
    @param scale:
    @return:
    """
    pub = rospy.Publisher(topic, OccupancyStamped, queue_size=1)
    i = 0
    while pub.get_num_connections() == 0:
        i += 1
        if i % 10 == 0:
            rospy.loginfo("Waiting for publisher to connect to topic: {}".format(topic))
        rospy.sleep(0.1)
    pub.publish(conversions.vox_to_occupancy_stamped(vg,
                                                     dim=vg.shape[1],
                                                     scale=scale,
                                                     frame_id=frame_id))


def publish_object_transform():
    """
    Publishes static ransform so shapes appear upright in rviz
    Note: This should not be necessary if starting ros using
    `roslaunch bsaund_shape_completion shape_completion
    """
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "object"
    q = tf_conversions.transformations.quaternion_from_euler(1.57,0,0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    rospy.sleep(1)
    br.sendTransform(t)

