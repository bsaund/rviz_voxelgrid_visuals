from unittest import TestCase
import numpy as np
import tensorflow as tf
from rviz_voxelgrid_visuals.conversions import vox_to_pointcloud2_msg, pointcloud2_msg_to_vox


class TestConversions(TestCase):

    def test_vox_to_pointcloud2_msg(self):
        vg = np.random.rand(64, 64, 64) > 0.5

        vg = tf.convert_to_tensor(vg)
        msg = vox_to_pointcloud2_msg(vg)
        vg_2 = pointcloud2_msg_to_vox(msg)
        self.assertTrue((vg == vg_2).numpy().all())

        vg_2 = pointcloud2_msg_to_vox(msg, scale=0.5)
        self.assertFalse((vg == vg_2).numpy().all())

        msg = vox_to_pointcloud2_msg(vg, scale=0.5)
        vg_2 = pointcloud2_msg_to_vox(msg, scale=0.5)
        self.assertTrue((vg == vg_2).numpy().all())

        msg = vox_to_pointcloud2_msg(vg, origin=(1.2, 2.3, 3.4))
        vg_2 = pointcloud2_msg_to_vox(msg, origin=(1.2, 2.3, 3.4))
        self.assertTrue((vg == vg_2).numpy().all())
        vg_2 = pointcloud2_msg_to_vox(msg)
        self.assertFalse((vg == vg_2).numpy().all())

        msg = vox_to_pointcloud2_msg(vg, scale=0.5, origin=(1.2, 2.3, 3.4))
        vg_2 = pointcloud2_msg_to_vox(msg, scale=0.5, origin=(1.2, 2.3, 3.4))
        self.assertTrue((vg == vg_2).numpy().all())
        # self.assertFalse(True)
