# import rospy
from mps_shape_completion_msgs.msg import OccupancyStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np


def vox_to_float_array(voxel_grid, dim):
    out_msg = Float32MultiArray()
    out_msg.data = voxel_grid.astype(np.float32).flatten().tolist()
    out_msg.layout.dim.append(MultiArrayDimension(label='x', size=dim, stride=dim*dim*dim))
    out_msg.layout.dim.append(MultiArrayDimension(label='y', size=dim, stride=dim*dim))
    out_msg.layout.dim.append(MultiArrayDimension(label='z', size=dim, stride=dim))
    return out_msg


def msg_to_vox(msg):
    return np.reshape(msg.data, tuple(d.size for d in msg.layout.dim))


def vox_to_occupancy_stamped(voxel_grid, dim, scale, frame_id, origin=(0,0,0)):
    msg = OccupancyStamped()
    msg.header.frame_id = frame_id
    msg.occupancy = vox_to_float_array(voxel_grid, dim)
    msg.scale = scale
    msg.origin.x = origin[0]
    msg.origin.y = origin[1]
    msg.origin.z = origin[2]
    return msg



def occupancyStamped_to_cubelist(occupancy_msg, color):
    """
    Takes an OccumancyStamped message with optional color and return a ros Marker as a cubelist
    INPUT: occupancy_msg: DATATYPE: OccupancyStamped
    INPUT: color: DATATYPE: std_msgs.msg.ColorRGBA
    OUTPUT: visualization_msgs.msg.Marker
    """

    if color is None:
        color = ColorRGBA()
        color.a = 1.0
        color.r = 0.5
        color.g = 0.5
        color.b = 0.5

    
    m = Marker()
    m.header = occupancy_msg.header

    m.color = color
    m.type = m.CUBE_LIST
    m.action = m.ADD
    m.pose.orientation.w = 1

    scale = occupancy_msg.scale
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    
    dim = occupancy_msg.occupancy.layout.dim
    data_offset = occupancy_msg.occupancy.layout.data_offset

    for i in range(dim[0].size):
        for j in range(dim[1].size):
            for k in range(dim[2].size):
                val = occupancy_msg.occupancy.data[data_offset + dim[1].stride * i + dim[2].stride * j + k]
                if val < 0.5:
                    continue
                p = Point()
                p.x = scale/2 + i*scale
                p.y = scale/2 + j*scale
                p.z = scale/2 + k*scale
                m.points.append(p)
                
    return m
