#include "rviz_voxelgrid_visuals/voxel_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

#include "rviz_voxelgrid_visuals/voxel_visual.h"

namespace rviz_voxelgrid_visuals {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
VoxelGridDisplay::VoxelGridDisplay() {
  // We inherit the unreliable property, but do not want to display it
  delete unreliable_property_;

  color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204), "Color of the voxel grid", this,
                                            SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha Multiple", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
                                            SLOT(updateColorAndAlpha()));

  binary_display_property_ = new rviz::BoolProperty(
      "Binary Display", true, "If checked, all voxels will have the same alpha", this, SLOT(updateColorAndAlpha()));

  cutoff_property_ = new rviz::FloatProperty(
      "Threshold", 0.5, "Voxels with values less than this will not be displayed", this, SLOT(updateColorAndAlpha()));

  hide_property_ = new rviz::BoolProperty("Hide", false, "Hide voxel grid", this, SLOT(updateColorAndAlpha()));
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void VoxelGridDisplay::onInitialize() {
  MFDClass::onInitialize();
  visual_.reset(new DenseVoxelGridVisual(context_->getSceneManager(), scene_node_));
  updateColorAndAlpha();
}

VoxelGridDisplay::~VoxelGridDisplay() = default;

void VoxelGridDisplay::reset() {
  MFDClass::reset();
  visual_->reset();
}

// Set the current color and alpha values for each visual.
void VoxelGridDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual_->setHidden(hide_property_->getBool());
  visual_->setBinaryDisplay(binary_display_property_->getBool());
  visual_->setColor(color.r, color.g, color.b, alpha);
  visual_->setThreshold(cutoff_property_->getFloat());
  visual_->updatePointCloud();
}

void VoxelGridDisplay::processMessage(const rviz_voxelgrid_visuals_msgs::VoxelgridStamped::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

/*****************************************
 *  Sparse Voxelgrid Display
 *****************************************/
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
SparseVoxelGridDisplay::SparseVoxelGridDisplay() {
  // We inherit the unreliable property, but do not want to display it
  delete unreliable_property_;

  color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204), "Color of the voxel grid", this,
                                            SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha Multiple", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
                                            SLOT(updateColorAndAlpha()));

  binary_display_property_ = new rviz::BoolProperty(
      "Binary Display", true, "If checked, all voxels will have the same alpha", this, SLOT(updateColorAndAlpha()));

  cutoff_property_ = new rviz::FloatProperty(
      "Threshold", 0.5, "Voxels with values less than this will not be displayed", this, SLOT(updateColorAndAlpha()));

  hide_property_ = new rviz::BoolProperty("Hide", false, "Hide voxel grid", this, SLOT(updateColorAndAlpha()));
}

void SparseVoxelGridDisplay::onInitialize() {
  MFDClass::onInitialize();
  visual_.reset(new SparseVoxelGridVisual(context_->getSceneManager(), scene_node_));
  updateColorAndAlpha();
}

SparseVoxelGridDisplay::~SparseVoxelGridDisplay() = default;

void SparseVoxelGridDisplay::reset() {
  MFDClass::reset();
  visual_->reset();
}

void SparseVoxelGridDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual_->setHidden(hide_property_->getBool());
  visual_->setBinaryDisplay(binary_display_property_->getBool());
  visual_->setColor(color.r, color.g, color.b, alpha);
  visual_->setThreshold(cutoff_property_->getFloat());
  visual_->updatePointCloud();
}

void SparseVoxelGridDisplay::processMessage(const rviz_voxelgrid_visuals_msgs::SparseVoxelgridStamped::ConstPtr& msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }
  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // end namespace rviz_voxelgrid_visuals

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
PLUGINLIB_EXPORT_CLASS(rviz_voxelgrid_visuals::VoxelGridDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(rviz_voxelgrid_visuals::SparseVoxelGridDisplay, rviz::Display)
