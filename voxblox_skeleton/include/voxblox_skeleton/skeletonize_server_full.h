/*
 * skeletonize_server.h
 *
 *  Created on: Aug 22, 2019
 *      Author: Leonid Bulyga
 */

#ifndef VOXBLOX_SKELETON_SKELETONIZE_SERVER_FULL_H_
#define VOXBLOX_SKELETON_SKELETONIZE_SERVER_FULL_H_

#include <ros/ros.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>

#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

#include "voxblox_skeleton/io/skeleton_io.h"
#include "voxblox_skeleton/ros/skeleton_vis.h"
#include "voxblox_skeleton/skeleton_generator.h"

namespace voxblox {

class SkeletonizeServerFull {
 public:
  SkeletonizeServerFull(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void init();
  void init_skeleton();
  void skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* skeleton,
                   std::vector<float>* distances);
  void esdf_update_cb(const std_msgs::EmptyConstPtr);

  EsdfServer* esdf_server_;
  SkeletonGenerator skeleton_generator_;

 protected:
  bool verbose;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string frame_id_;
  bool update_esdf;
  bool skeletonizer_visualize_;

  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;
  ros::Publisher skeletonizer_pub_;
  ros::Subscriber esdf_sub_;


};

}

#endif /* VOXBLOX_SKELETON_SKELETONIZE_SERVER_FULL_H_ */
