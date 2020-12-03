# import rospy
from rviz_voxelgrid_visuals_msgs.msg import VoxelgridStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import ColorRGBA, Header
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np


def vox_to_float_array(voxel_grid, dim=None):
    if dim is None:
        dim = voxel_grid.shape[0]
    out_msg = Float32MultiArray()
    out_msg.data = voxel_grid.astype(np.float32).flatten().tolist()
    out_msg.layout.dim.append(MultiArrayDimension(label='x', size=dim, stride=dim * dim))
    out_msg.layout.dim.append(MultiArrayDimension(label='y', size=dim, stride=dim))
    out_msg.layout.dim.append(MultiArrayDimension(label='z', size=dim, stride=1))
    return out_msg


def msg_to_vox(msg):
    return np.reshape(msg.data, tuple(d.size for d in msg.layout.dim))


def vox_to_voxelgrid_stamped(voxel_grid, scale, frame_id, dim=None, origin=(0, 0, 0)):
    """
    @param voxel_grid: 3D Cubic Voxelgrid in either numpy or tensorflow
    @param scale: side dimension of each voxel
    @param frame_id: name of frame of voxelgrid
    @param dim: dimension of voxelgrid (optional, will be inferred if possible)
    @param origin: origin of the voxelgrid in the frame from lower, bottom, left voxel
    @return:
    """
    msg = VoxelgridStamped()
    msg.header.frame_id = frame_id
    msg.occupancy = vox_to_float_array(voxel_grid, dim)
    msg.scale = scale
    msg.origin.x = origin[0]
    msg.origin.y = origin[1]
    msg.origin.z = origin[2]
    return msg


def vox_to_pointcloud(voxel_grid, scale=1.0, origin=(0, 0, 0), threshold=0.5):
    """
    Converts a 3D voxelgrid into a 3D set of points for each voxel with value above threshold
    @param voxelgrid: (opt 1 x) X x Y x Z (opt x 1) voxelgrid
    @param scale:
    @param origin: origin in voxel coorindates
    @param threshold:
    @return:
    """
    pts = np.argwhere(np.squeeze(voxel_grid) > threshold)
    return (np.array(pts) - origin + 0.5) * scale


def vox_to_pointcloud2_msg(voxel_grid, scale=1.0, frame="world", origin=(0, 0, 0), threshold=0.5):
    header = Header(frame_id=frame)

    pts = [pt for pt in vox_to_pointcloud(voxel_grid, scale=scale, origin=origin, threshold=threshold)]

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              ]
    return point_cloud2.create_cloud(header, fields, pts)


def voxelgrid_stamped_to_cubelist(occupancy_msg, color):
    """
    "Deprecated. Use RViz plugin and voxelgrid_stamped messages
    Takes an OccumancyStamped message with optional color and return a ros Marker as a cubelist
    INPUT: occupancy_msg: DATATYPE: VoxelgridStamped
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
                p.x = scale / 2 + i * scale
                p.y = scale / 2 + j * scale
                p.z = scale / 2 + k * scale
                m.points.append(p)

    return m
