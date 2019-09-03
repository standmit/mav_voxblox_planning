#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>

#include "voxblox_skeleton_planner/skeleton_global_planner.h"

namespace mav_planning {

SkeletonGlobalPlanner::SkeletonGlobalPlanner(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      voxblox_server_(nh_, nh_private_),
      skeleton_graph_planner_(nh_, nh_private_),
      skeleton_generator_() {
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, std::string("map"));

  run_astar_esdf = nh_private_.param("run_astar_esdf", false);
  run_astar_diagram = nh_private_.param("run_astar_diagram", true);
  run_astar_graph = nh_private_.param("run_astar_graph", true);
  shorten_graph = nh_private_.param("shorten_graph", true);
  smooth_path  = nh_private_.param("smooth_path", true);
  verbose_ = nh_private_.param("verbose", true);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "skeleton", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &SkeletonGlobalPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &SkeletonGlobalPlanner::publishPathCallback, this);

  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

  skeleton_generator_.setMinGvdDistance(constraints_.robot_radius);
  skeleton_generator_.setGenerateByLayerNeighbors(true);

  skeleton_planner_.setMinEsdfDistance(constraints_.robot_radius);

  path_shortener_.setConstraints(constraints_);

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMapDistanceCallback(std::bind(
      &SkeletonGlobalPlanner::getMapDistance, this, std::placeholders::_1));

  voxblox_sub_ = nh_private_.subscribe("voxblox", 1, &SkeletonGlobalPlanner::voxblox_cb, this);
}

inline bool remove_file(const std::string& filename) {
  return (remove(filename.c_str()) == 0);
}

inline void remove_file_ros(const std::string& filename) {
  if (not remove_file(filename))
    ROS_ERROR("Failed to remove [%s] file", filename.c_str());
}

void SkeletonGlobalPlanner::voxblox_cb(const mav_msgs::DoubleStringConstPtr msg) {
  voxblox::Layer<voxblox::SkeletonVoxel>* skeleton_layer;
  try {
    // Load a file.
    if (msg->skeleton.empty()) {
      const std::string err_mes("Couldn't load map, empty filename.");
      ROS_ERROR("%s", err_mes.c_str());
      throw err_mes;
    }

    if (not msg->graph.empty())
      sparse_graph_path_ = msg->graph;

    if (!voxblox_server_.loadMap(msg->skeleton)) {
      ROS_ERROR("Couldn't load ESDF map!");
      throw "Couldn't load ESDF map!";
    }

    std::shared_ptr<voxblox::EsdfMap> esdf_map = voxblox_server_.getEsdfMapPtr();
    CHECK(esdf_map);

    ROS_INFO_COND(verbose_,
        "Size: %f VPS: %zu",
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

    // Also make a new skeleton layer and load it.
    // Make this as an unmanaged raw pointer, since we'll give it to skeleton
    // generator to own.
    skeleton_layer =
        new voxblox::Layer<voxblox::SkeletonVoxel>(
            voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
            voxblox_server_.getEsdfMapPtr()
                ->getEsdfLayerPtr()
                ->voxels_per_side());

    if (!voxblox::io::LoadBlocksFromFile<voxblox::SkeletonVoxel>(
            msg->graph,
            voxblox::Layer<
                voxblox::SkeletonVoxel>::BlockMergingStrategy::kReplace,
            true, skeleton_layer)) {
      ROS_ERROR("Coudln't load skeleton layer.");
    }

    // Now set up the skeleton generator.
    skeleton_generator_.setEsdfLayer(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

    skeleton_generator_.setSkeletonLayer(skeleton_layer);

    // Set up the A* planners.
    skeleton_planner_.setSkeletonLayer(skeleton_generator_.getSkeletonLayerPtr());
    skeleton_planner_.setEsdfLayer(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

    // Set up skeleton graph planner.
    skeleton_graph_planner_.setEsdfLayer(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

    // Set up shortener.
    path_shortener_.setEsdfLayer(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

    loco_smoother_.setMinCollisionCheckResolution(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size());

    generateSparseGraph();

    remove_file_ros(msg->skeleton);
    remove_file_ros(msg->graph);
  }
  catch (...) {
    remove_file_ros(msg->skeleton);
    remove_file_ros(msg->graph);
  }

  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
  }
}

void SkeletonGlobalPlanner::generateSparseGraph() {
  ROS_INFO_COND(verbose_, "About to generate skeleton graph.");
  skeleton_generator_.updateSkeletonFromLayer();
  ROS_INFO_COND(verbose_, "Re-populated from layer.");

  if (!sparse_graph_path_.empty() &&
      skeleton_generator_.loadSparseGraphFromFile(sparse_graph_path_)) {
	ROS_INFO_STREAM_COND(verbose_, "Loaded sparse graph from file: " << sparse_graph_path_);
  } else {
    skeleton_generator_.generateSparseGraph();
    ROS_INFO_COND(verbose_, "Generated skeleton graph.");
  }

  if (visualize_) {
    voxblox::Pointcloud pointcloud;
    std::vector<float> distances;
    skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(
        &pointcloud, &distances);

    // Publish the skeleton.
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
    voxblox::pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
    ptcloud_pcl.header.frame_id = frame_id_;
    skeleton_pub_.publish(ptcloud_pcl);

    // Now visualize the graph.
    const voxblox::SparseSkeletonGraph& graph =
        skeleton_generator_.getSparseGraph();
    visualization_msgs::MarkerArray marker_array;
    voxblox::visualizeSkeletonGraph(graph, frame_id_, &marker_array);
    sparse_graph_pub_.publish(marker_array);
  }

  // Set up the graph planner.
  mav_trajectory_generation::timing::Timer kd_tree_init("plan/graph/setup");
  skeleton_graph_planner_.setSparseGraph(&skeleton_generator_.getSparseGraph());
  kd_tree_init.Stop();

  ROS_INFO_STREAM_COND(verbose_, "Generation timings: " << std::endl
										 	 	 	 	<< voxblox::timing::Timing::Print());
}

bool SkeletonGlobalPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

  try {
	  listener.transformPose(frame_id_, request.start_pose, request.start_pose);
	  listener.transformPose(frame_id_, request.goal_pose, request.goal_pose);
  }
  catch( const tf::TransformException  &e )
  {
      ROS_ERROR( "%s", e.what() );
      return false;
  }

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  ROS_INFO("Planning path.");

  /*
  if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Start pose occupied!");
    return false;
  }
  if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Goal pose occupied!");
    return false;
  }
  */

  voxblox::Point start_point =
      start_pose.position_W.cast<voxblox::FloatingPoint>();
  voxblox::Point goal_point =
      goal_pose.position_W.cast<voxblox::FloatingPoint>();

  visualization_msgs::MarkerArray marker_array;

  if (run_astar_esdf) {
    // First, run just the ESDF A*...
    voxblox::AlignedVector<voxblox::Point> esdf_coordinate_path;
    mav_trajectory_generation::timing::Timer astar_esdf_timer(
        "plan/astar_esdf");
    bool success = skeleton_planner_.getPathInEsdf(start_point, goal_point,
                                                   &esdf_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector esdf_path;
    convertCoordinatePathToPath(esdf_coordinate_path, &esdf_path);
    double path_length = computePathLength(esdf_path);
    int num_vertices = esdf_path.size();
    astar_esdf_timer.Stop();
    ROS_INFO("ESDF A* Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          esdf_path, frame_id_, mav_visualization::Color::Yellow(),
          "astar_esdf", 0.1));
    }
  }

  if (run_astar_diagram) {
    voxblox::AlignedVector<voxblox::Point> diagram_coordinate_path;
    mav_trajectory_generation::timing::Timer astar_diag_timer(
        "plan/astar_diag");
    bool success = skeleton_planner_.getPathUsingEsdfAndDiagram(
        start_point, goal_point, &diagram_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector diagram_path;
    convertCoordinatePathToPath(diagram_coordinate_path, &diagram_path);
    double path_length = computePathLength(diagram_path);
    int num_vertices = diagram_path.size();
    astar_diag_timer.Stop();
    ROS_INFO("Diag A* Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          diagram_path, frame_id_, mav_visualization::Color::Purple(),
          "astar_diag", 0.1));
    }

    if (shorten_graph) {
      mav_trajectory_generation::timing::Timer shorten_timer(
          "plan/astar_diag/shorten");
      mav_msgs::EigenTrajectoryPointVector short_path;
      path_shortener_.shortenPath(diagram_path, &short_path);
      path_length = computePathLength(short_path);
      num_vertices = short_path.size();
      ROS_INFO("Diagram Shorten Success? %d Path length: %f Vertices: %d",
               success, path_length, num_vertices);
      if (visualize_) {
        marker_array.markers.push_back(createMarkerForPath(
            short_path, frame_id_, mav_visualization::Color::Pink(),
            "short_astar_plan", 0.1));
      }
      shorten_timer.Stop();
    }
  }

  if (run_astar_graph) {
    mav_msgs::EigenTrajectoryPointVector graph_path;
    mav_trajectory_generation::timing::Timer graph_timer("plan/graph");
    skeleton_graph_planner_.setShortenPath(false);
    bool success = skeleton_graph_planner_.getPathBetweenWaypoints(
        start_pose, goal_pose, &graph_path);
    double path_length = computePathLength(graph_path);
    int num_vertices = graph_path.size();
    graph_timer.Stop();
    ROS_INFO("Graph Planning Success? %d Path length: %f Vertices: %d", success,
             path_length, num_vertices);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          graph_path, frame_id_, mav_visualization::Color::Blue(), "graph_plan",
          0.1));
    }

    last_waypoints_ = graph_path;

    if (shorten_graph) {
      mav_trajectory_generation::timing::Timer shorten_timer(
          "plan/graph/shorten");
      mav_msgs::EigenTrajectoryPointVector short_path;
      success = path_shortener_.shortenPath(graph_path, &short_path);
      path_length = computePathLength(short_path);
      num_vertices = short_path.size();
      ROS_INFO("Shorten Success? %d Path length: %f Vertices: %d", success,
               path_length, num_vertices);
      shorten_timer.Stop();

      if (visualize_) {
        marker_array.markers.push_back(createMarkerForPath(
            short_path, frame_id_, mav_visualization::Color::Green(),
            "short_plan", 0.1));
      }

      last_waypoints_ = short_path;

      if (smooth_path) {
        mav_msgs::EigenTrajectoryPointVector loco_path;
        mav_trajectory_generation::timing::Timer loco_timer("plan/graph/loco");
        loco_smoother_.setResampleVisibility(true);
        loco_smoother_.setAddWaypoints(false);
        loco_smoother_.setNumSegments(5);
        loco_smoother_.getPathBetweenWaypoints(short_path, &loco_path);

        loco_timer.Stop();

        last_waypoints_ = loco_path;

        if (visualize_) {
          marker_array.markers.push_back(createMarkerForPath(
              loco_path, frame_id_, mav_visualization::Color::Teal(),
              "loco_plan", 0.1));
        }
      }
    }
  }

  last_start_pose = request.start_pose;
  last_goal = request.goal_pose;

  if (visualize_) {
    path_marker_pub_.publish(marker_array);
  }

  if (verbose_)
	  ROS_INFO_STREAM("All timings: "
					  << std::endl
					  << mav_trajectory_generation::timing::Timing::Print());
  return true;
}

void SkeletonGlobalPlanner::convertCoordinatePathToPath(
    const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) const {
  CHECK_NOTNULL(path);
  path->clear();
  path->reserve(coordinate_path.size());

  for (const voxblox::Point& voxblox_point : coordinate_path) {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = voxblox_point.cast<double>();
    path->push_back(point);
  }
}

double SkeletonGlobalPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    return 0.0;
  }
  return distance;
}

bool SkeletonGlobalPlanner::publishPathCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  ROS_INFO("Publishing waypoints.");

  geometry_msgs::PoseArray pose_array;
  pose_array.poses.reserve(last_waypoints_.size());
  tf::Quaternion start_orientation;
  tf::quaternionMsgToTF(last_start_pose.pose.orientation, start_orientation);
  tf::Quaternion goal_orientation;
  tf::quaternionMsgToTF(last_goal.pose.orientation, goal_orientation);
  unsigned long point_i = 0;
  for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
    geometry_msgs::PoseStamped pose_stamped;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
    tf::quaternionTFToMsg(
		tf::slerp(
			start_orientation,
			goal_orientation,
			static_cast<float>(point_i++) / last_waypoints_.size()
		),
		pose_stamped.pose.orientation
	);
    pose_array.poses.push_back(pose_stamped.pose);
  }
  pose_array.poses.push_back(last_goal.pose);

  pose_array.header.frame_id = frame_id_;
  waypoint_list_pub_.publish(pose_array);
  return true;
}

}  // namespace mav_planning
