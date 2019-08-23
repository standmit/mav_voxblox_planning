#include <sys/stat.h>
#include <voxblox_skeleton/skeletonize_server_full.h>
#include "std_msgs/Empty.h"

namespace voxblox {

SkeletonizeServerFull::SkeletonizeServerFull(
		const ros::NodeHandle& nh,
		const ros::NodeHandle& nh_private
		):
			nh_(nh),
			nh_private_(nh_private),
			frame_id_("map"),
			update_esdf(false),
			verbose(true),
			skeletonizer_visualize_(true),
			esdf_server_(NULL) {
	init();
}

void SkeletonizeServerFull::init(){
	nh_private_.param("frame_id", frame_id_, frame_id_);
	nh_private_.param("update_esdf", update_esdf, update_esdf);
	nh_private_.param("verbose", verbose, verbose);
	nh_private_.param("skeletonizer_visualize", skeletonizer_visualize_, skeletonizer_visualize_);

	if (skeletonizer_visualize_) {
		skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
			"skeleton", 1, true);
		sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
			"sparse_graph", 1, true);
	}

    skeletonizer_pub_ = nh_private_.advertise<std_msgs::Empty>(
    		"skeletonizer_update_trigger", 1, true);
    esdf_sub_ = nh_private_.subscribe(
    		"esdf_update_trigger", 1, &SkeletonizeServerFull::esdf_update_cb, this);
    esdf_server_ = new EsdfServer (nh_, nh_private_);
}

void SkeletonizeServerFull::init_skeleton() {


  esdf_server_->disableIncrementalUpdate();
  if (update_esdf ||
      esdf_server_->getEsdfMapPtr()
              ->getEsdfLayerPtr()
              ->getNumberOfAllocatedBlocks() == 0) {
    const bool full_euclidean_distance = true;
    esdf_server_->updateEsdfBatch(full_euclidean_distance);
  }

  // Visualize all parts.
  esdf_server_->updateMesh();
  esdf_server_->publishPointclouds();
  esdf_server_->publishMap();

  ROS_INFO_COND(verbose, "Finished updating ESDF.");

  // Skeletonize????
  voxblox::Pointcloud pointcloud;
  std::vector<float> distances;
  skeletonize(esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(), &pointcloud,
              &distances);

  if (skeletonizer_visualize_) {
  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = frame_id_;
  skeleton_pub_.publish(ptcloud_pcl);
  }
}

void SkeletonizeServerFull::skeletonize(Layer<EsdfVoxel>* esdf_layer,
                                   voxblox::Pointcloud* pointcloud,
                                   std::vector<float>* distances) {
  skeleton_generator_.setEsdfLayer(esdf_layer);

  FloatingPoint min_separation_angle =
      skeleton_generator_.getMinSeparationAngle();
  nh_private_.param("min_separation_angle", min_separation_angle,
                    min_separation_angle);
  skeleton_generator_.setMinSeparationAngle(min_separation_angle);
  bool generate_by_layer_neighbors =
      skeleton_generator_.getGenerateByLayerNeighbors();
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors,
                    generate_by_layer_neighbors);
  skeleton_generator_.setGenerateByLayerNeighbors(generate_by_layer_neighbors);

  int num_neighbors_for_edge = skeleton_generator_.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge,
                    num_neighbors_for_edge);
  skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge);

  FloatingPoint min_gvd_distance = skeleton_generator_.getMinGvdDistance();
  nh_private_.param("min_gvd_distance", min_gvd_distance, min_gvd_distance);
  skeleton_generator_.setMinGvdDistance(min_gvd_distance);

  skeleton_generator_.generateSkeleton();
  skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(pointcloud,
                                                                   distances);
  ROS_INFO_COND(verbose, "Finished generating skeleton.");

  skeleton_generator_.generateSparseGraph();
  ROS_INFO_COND(verbose, "Finished generating sparse graph.");

  ROS_INFO_STREAM_COND(verbose, "Total Timings: " << std::endl << timing::Timing::Print());

  if (skeletonizer_visualize_) {
  const SparseSkeletonGraph& graph = skeleton_generator_.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id_, &marker_array);
  sparse_graph_pub_.publish(marker_array);
  }
}



void SkeletonizeServerFull::esdf_update_cb(const std_msgs::EmptyConstPtr /*msg*/){
	try {
		init_skeleton();
	}
	catch (...) {
		ROS_ERROR("Skeletonize failure.");
		return;
	}
	static std_msgs::Empty sk_msg;
	skeletonizer_pub_.publish(sk_msg);
}

}  // namespace voxblox


