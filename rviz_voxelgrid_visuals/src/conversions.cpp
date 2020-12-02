//
// Created by bsaund on 12/1/20.
//
#include "rviz_voxelgrid_visuals/conversions.h"
#include <iostream>

std_msgs::Float32MultiArray RVV::voxelgridToFloatArray() {
  std::cout << "Called voxelgridToFloatArray()\n";
  return std_msgs::Float32MultiArray();

  //rviz_voxelgrid_visuals_msgs::VoxelgridStamped GVP::denseGridToMsg(const DenseGrid& g, const std::string& frame) {
//  rviz_voxelgrid_visuals_msgs::VoxelgridStamped msg;
//  std::cout << "Converting to msg\n";
//  auto dims = g.getDimensions();
//  std::vector<float> data(dims.x * dims.y * dims.z);
//  Eigen::TensorMap<Eigen::Tensor<float, 3>> t(data.data(), dims.x, dims.y, dims.z);
//
//  for (auto coord : g.getOccupiedCoords()) {
//    t(coord.x, coord.y, coord.z) = 1.0;
//  }
//
////  for(auto val: data){
////    if(val>=0.5){
////      std::cout << val << ", ";
////    }
////  }
//
//  msg.header.frame_id = frame;
//  msg.scale = g.getVoxelSideLength();
//  msg.occupancy.data = data;
//
//  std_msgs::MultiArrayDimension dx;
//  dx.label = "x";
//  dx.size = dims.x;
//  dx.stride = 1;
//  std_msgs::MultiArrayDimension dy;
//  dy.label = "y";
//  dy.size = dims.y;
//  dy.stride = dims.x;
//  std_msgs::MultiArrayDimension dz;
//  dz.label = "z";
//  dz.size = dims.z;
//  dz.stride = dims.x * dims.y;
//
//  msg.occupancy.layout.dim.push_back(dx);
//  msg.occupancy.layout.dim.push_back(dy);
//  msg.occupancy.layout.dim.push_back(dz);
//
//  return msg;
//}
}