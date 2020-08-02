#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/point_cloud.h>
#include <std_msgs/Float32MultiArray.h>

#include "voxel_visual.h"

namespace mps_shape_completion_visualization
{

// BEGIN_TUTORIAL
    VoxelGridVisual::VoxelGridVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    {
        scene_manager_ = scene_manager;

        frame_node_ = parent_node->createChildSceneNode();

        voxel_grid_.reset(new rviz::PointCloud());
        voxel_grid_->setRenderMode(rviz::PointCloud::RM_BOXES);
        frame_node_->attachObject(voxel_grid_.get());
    }

    VoxelGridVisual::~VoxelGridVisual()
    {
        scene_manager_->destroySceneNode( frame_node_ );
    }

    void VoxelGridVisual::reset()
    {
        latest_msg = mps_shape_completion_msgs::OccupancyStamped();
        voxel_grid_->clear();
    }


    void VoxelGridVisual::setMessage( const mps_shape_completion_msgs::OccupancyStamped::ConstPtr& msg)
    {
        latest_msg = *msg;
        updatePointCloud();
    }

    void VoxelGridVisual::updatePointCloud()
    {
        if(hidden_)
        {
            voxel_grid_->clear();
            return;
        }

        if(latest_msg.occupancy.layout.dim.size() == 0)
        {
            return;
        }

        
        double scale = latest_msg.scale;
        voxel_grid_->setDimensions(scale, scale, scale);


        const std::vector<float> data = latest_msg.occupancy.data;
        const std::vector<std_msgs::MultiArrayDimension> dims = latest_msg.occupancy.layout.dim;
        int data_offset = latest_msg.occupancy.layout.data_offset;

        std::vector< rviz::PointCloud::Point> points;
        for(int i=0; i<dims[0].size; i++)
        {
            for(int j=0; j<dims[1].size; j++)
            {
                for(int k=0; k<dims[2].size; k++)
                {
                    float val = data[data_offset + dims[1].stride * i + dims[2].stride * j + k];
                    if(val < threshold_)
                    {
                        continue;
                    }
                    
                    rviz::PointCloud::Point p;
                    p.position.x = scale/2 + i*scale + latest_msg.origin.x;
                    p.position.y = scale/2 + j*scale + latest_msg.origin.y;
                    p.position.z = scale/2 + k*scale + latest_msg.origin.z;

                    if(binary_display_)
                    {
                        val = 1.0;
                    }

                    p.setColor(r_, g_, b_, std::min(val*a_, (float)1.0));
                    
                    points.push_back(p);
                }
            }
        }
        
        voxel_grid_->clear();

        //The per-point alpha setting is not great with alpha=1, so in
        // certain cases do not use it
        bool use_per_point = !(a_ >= 1.0 && binary_display_);
        voxel_grid_->setAlpha(a_, use_per_point);
        
        voxel_grid_->addPoints(&points.front(), points.size());

    }



// Position and orientation are passed through to the SceneNode.
    void VoxelGridVisual::setFramePosition(const Ogre::Vector3& position )
    {
        frame_node_->setPosition( position );
    }

    void VoxelGridVisual::setFrameOrientation(const Ogre::Quaternion& orientation )
    {
        frame_node_->setOrientation( orientation );
    }

    void VoxelGridVisual::setColor(float r, float g, float b, float a)
    {
        r_ = r;
        g_ = g;
        b_ = b;
        a_ = a;
    }

    void VoxelGridVisual::setBinaryDisplay(bool binary_display)
    {
        binary_display_ = binary_display;
    }

    void VoxelGridVisual::setThreshold(float threshold)
    {
        threshold_ = threshold;
    }

    void VoxelGridVisual::setHidden(bool hidden)
    {
        hidden_ = hidden;
    }

} // end namespace mps_shape_completion_visualization
