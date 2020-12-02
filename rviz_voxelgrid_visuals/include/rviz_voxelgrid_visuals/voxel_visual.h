#ifndef VOXEL_VISUAL_H
#define VOXEL_VISUAL_H

#include <rviz_voxelgrid_visuals_msgs/VoxelgridStamped.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

//namespace Ogre {
//class Vector3;
//class Quaternion;
//}  // namespace Ogre

namespace rviz {
class PointCloud;
}

namespace rviz_voxelgrid_visuals {

class VoxelGridVisual {
 public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  VoxelGridVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~VoxelGridVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const rviz_voxelgrid_visuals_msgs::VoxelgridStamped::ConstPtr& msg);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor(float r, float g, float b, float a);

  void setBinaryDisplay(bool use_global_alpha);
  void setThreshold(float threshold);
  void setHidden(bool hidden);

  // Rerenders the point cloud from the list of points and UI-selected properties
  void updatePointCloud();

  void reset();

 private:
  // A local copy of the message is stored so that the voxelgrid can be
  // regenerated if the user changes the input
  rviz_voxelgrid_visuals_msgs::VoxelgridStamped latest_msg;

  // The visible voxel grid ogre object
  boost::shared_ptr<rviz::PointCloud> voxel_grid_;

  // A SceneNode whose pose is set to match the coordinate frame of
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // User settings
  float r_, g_, b_, a_;
  bool binary_display_;
  float threshold_;
  bool hidden_;
};

}  // end namespace rviz_voxelgrid_visuals

#endif  // VOXEL_VISUAL_H
