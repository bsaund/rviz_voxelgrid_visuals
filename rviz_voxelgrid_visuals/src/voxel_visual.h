#ifndef VOXEL_VISUAL_H
#define VOXEL_VISUAL_H

#include <mps_shape_completion_msgs/OccupancyStamped.h>


namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class PointCloud;
}

namespace mps_shape_completion_visualization
{

class VoxelGridVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  VoxelGridVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~VoxelGridVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const mps_shape_completion_msgs::OccupancyStamped::ConstPtr& msg);

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor( float r, float g, float b, float a );

  void setBinaryDisplay(bool use_global_alpha);
  void setThreshold(float threshold);
  void setHidden(bool hidden);

  //Rerenders the point cloud from the list of points and UI-selected properties
  void updatePointCloud();

  void reset();


private:

  // A local copy of the message is stored so that the voxelgrid can be
  // regenerated if the user changes the input
  mps_shape_completion_msgs::OccupancyStamped latest_msg;
  
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

} // end namespace mps_shape_completion_visualization

#endif // VOXEL_VISUAL_H
