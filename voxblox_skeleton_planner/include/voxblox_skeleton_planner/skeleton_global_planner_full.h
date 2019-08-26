#ifndef VOXBLOX_SKELETON_PLANNER_SKELETON_GLOBAL_PLANNER_H_
#define VOXBLOX_SKELETON_PLANNER_SKELETON_GLOBAL_PLANNER_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <mav_msgs/conversions.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>
#include <voxblox_planning_common/path_shortening.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_planner.h>
#include <voxblox_skeleton/skeletonize_server_full.h>
#include <voxblox_skeleton/sparse_graph_planner.h>
#include "voxblox_skeleton_planner/skeleton_graph_planner.h"

namespace mav_planning {

class SkeletonGlobalPlannerFull {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SkeletonGlobalPlannerFull(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
  virtual ~SkeletonGlobalPlannerFull() {}

  void init();
  void setupGraphPlanner();

  bool plannerServiceCallback(
      mav_planning_msgs::PlannerServiceRequest& request,
      mav_planning_msgs::PlannerServiceResponse& response);
  bool publishPathCallback(std_srvs::EmptyRequest& request,
                           std_srvs::EmptyResponse& response);
  void convertCoordinatePathToPath(
      const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path) const;
  double getMapDistance(const Eigen::Vector3d& position) const;

 protected:
  bool run_astar_esdf;
  bool run_astar_diagram;
  bool run_astar_graph;
  bool shorten_graph;
  bool smooth_path;
  bool verbose_;
  geometry_msgs::PoseStamped last_start_pose, last_goal;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher path_marker_pub_;
  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;
  ros::Publisher path_pub_;
  ros::Publisher waypoint_list_pub_;

  ros::Subscriber skeletonizer_sub_;

  void skeletonizer_update_cb(const std_msgs::EmptyConstPtr);

  ros::ServiceServer planner_srv_;
  ros::ServiceServer path_pub_srv_;

  tf::TransformListener listener;

  // Settings for physical constriants.
  mav_planning::PhysicalConstraints constraints_;

  std::string frame_id_;
  bool visualize_;

  voxblox::SkeletonizeServerFull skeletonizer_;

  // Planners of all sorts.
  voxblox::SkeletonAStar skeleton_planner_;
  SkeletonGraphPlanner skeleton_graph_planner_;
  EsdfPathShortener path_shortener_;
  LocoSmoother loco_smoother_;

  // Waypoints
  mav_msgs::EigenTrajectoryPointVector last_waypoints_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_SKELETON_PLANNER_SKELETON_GLOBAL_PLANNER_H_
