# Visualization Tools for Voxelgrids in RViz

This RViz plugin and associated tools enable easy viewing of Voxelgrids in RViz. 

You can:
1. change the scale, origin, and frame 
1. change the color, and opacity (alpha) directly from RViz without needing to republish or edit code
1. view data as binary or with independent alpha per voxel, again without needing to republish
1. hide/show the voxelgrid from RViz without needing to republish

This adds a new display-type to RViz which enables voxelgrid messages

![Panel Image](/rviz_voxelgrid_visuals/imgs/panel.png)
![RViz_image](/rviz_voxelgrid_visuals/imgs/expected_tutorial.png)


# Getting started

Prereqs:
- ROS, with RViz
- python 2 (not tested but might work with python 3)

Installation:
1. Clone this repo into your catkin workspace
2. Build `catkin_make` or `catkin build`
3. Re-source your catkin `devel` workspace!
4. Launch `rviz` and open the `example_voxelgrid.rviz` config file. Be sure you have re-sourced your `devel` workspace first, from the same terminal where you launch rviz. 
5. Run the `quick_publish_demo.py` script: `rosrun rviz_voxelgrid_visuals quick_publish_demo.py`


You should see the images above.
Try changing the color, opacity, and hiding and re-showing:
![RViz_image](/rviz_voxelgrid_visuals/imgs/expected_tutorial_1.png)
![RViz_image](/rviz_voxelgrid_visuals/imgs/expected_tutorial_2.png)


Check out the script for how to publish in your projects: e.g.
```
pub_1.publish(conversions.vox_to_voxelgrid_stamped(voxelgrid, # Numpy or Tensorflow
                                                   scale=0.01, # Each voxel is a 1cm cube
                                                   frame_id='world', # In frame "world", same as rviz fixed frame
                                                   origin=(0,0,0))) # Bottom left corner

```
