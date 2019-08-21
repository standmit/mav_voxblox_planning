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

#include "std_msgs/String.h"
#include "mav_msgs/DoubleString.h"

#include <sys/stat.h>

namespace voxblox {

class SkeletonizerNode {
 public:
  SkeletonizerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      :
    	nh_(nh),
    	nh_private_(nh_private),
        frame_id_("map"),
		update_esdf(false),
		verbose(true),
		esdf_server_(NULL) {}

  void init();
  void init_skeleton(const std::string& output_skeleton_path,
					 const std::string& output_graph_path);

  void skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* skeleton,
                   std::vector<float>* distances);
  void esdf_update_cb(const std_msgs::EmptyConstPtr msg);

 protected:
  bool verbose;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string frame_id_;
  std::string output_file_path_;
  bool update_esdf;
  uint8_t file_number_ = 0;
  std::string last_file_skeleton;
  std::string last_file_graph;

  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;
  ros::Publisher file_path_pub_;
  ros::Subscriber esdf_sub_;

  EsdfServer* esdf_server_;

  SkeletonGenerator skeleton_generator_;
};

void SkeletonizerNode::init(){
	nh_private_.param("frame_id", frame_id_, frame_id_);
	nh_private_.param("update_esdf", update_esdf, false);
	nh_private_.param("verbose", verbose, true);
	nh_private_.param("file_path", output_file_path_, std::string("/home/ubuntu/tmpfs_voxblox"));

    skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
        "skeleton", 1, true);
    sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "sparse_graph", 1, true);
    file_path_pub_ = nh_private_.advertise<mav_msgs::DoubleString>(
    		"skeleton_and_graph_file", 1, true);
    esdf_sub_ = nh_private_.subscribe(
    		"esdf_update_trigger",1 , &SkeletonizerNode::esdf_update_cb, this);
    esdf_server_ = new EsdfServer (nh_, nh_private_);
}

void SkeletonizerNode::init_skeleton(const std::string& output_skeleton_path,
		 	 	 	 	 	 	 	 const std::string& output_graph_path) {


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

  // Publish the skeleton.
  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = frame_id_;
  skeleton_pub_.publish(ptcloud_pcl);

  // Optionally save back to file.
  if (!output_skeleton_path.empty()) {
    // Put the TSDF, ESDF, and skeleton layer in the same bucket.
    if (esdf_server_->saveMap(output_skeleton_path)) {
      constexpr bool kClearFile = false;
      io::SaveLayer<SkeletonVoxel>(*skeleton_generator_.getSkeletonLayer(),
    		  	  	  	  	  	   output_skeleton_path, kClearFile);
      ROS_INFO_COND(verbose, "Output skeleton to: %s", output_skeleton_path.c_str());
    } else {
      ROS_ERROR("Couldn't output skeleton to: %s", output_skeleton_path.c_str());
    }
  }
  if (!output_graph_path.empty()) {
    if (skeleton_generator_.saveSparseGraphToFile(output_graph_path)) {
      ROS_INFO_COND(verbose, "Output graph to: %s", output_graph_path.c_str());
    } else {
      ROS_ERROR("Couldn't output graph to: %s",
    		  output_graph_path.c_str());
    }
  }
}

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>* esdf_layer,
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

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator_.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id_, &marker_array);
  sparse_graph_pub_.publish(marker_array);
}

inline bool remove_file(const std::string& filename) {
  return (remove(filename.c_str()) == 0);
}

inline void remove_file_ros(const std::string& filename) {
  if (not remove_file(filename))
    ROS_ERROR("Failed to remove [%s] file", filename.c_str());
}

inline bool file_exists(const std::string& filename) {
	struct stat buffer;
	return (stat(filename.c_str(), &buffer) == 0);
}

inline void delete_if_exist(const std::string& filename) {
	if (not filename.empty()) {
		if (file_exists(filename)) {
			remove_file_ros(filename);
		}
	}
}


void SkeletonizerNode::esdf_update_cb(const std_msgs::EmptyConstPtr /*msg*/){
	std::string output_skeleton_path, output_graph_path;
	output_skeleton_path = output_file_path_ + "/s" + std::to_string(this->file_number_) + ".vxblx";;
	output_graph_path = output_file_path_ + "/g" + std::to_string(this->file_number_) + ".vxblx";;

	try {
		init_skeleton(output_skeleton_path, output_graph_path);
	}
	catch (...) {
		ROS_ERROR("Skeletonize failure, broken files removed.");
		remove_file_ros(output_skeleton_path);
		remove_file_ros(output_graph_path);
		return;
	}

	delete_if_exist(last_file_skeleton);
	delete_if_exist(last_file_graph);

	mav_msgs::DoubleString out_msg;
	out_msg.skeleton = output_skeleton_path;
	out_msg.graph = output_graph_path;
	file_path_pub_.publish(out_msg);
	this->file_number_++;
	last_file_skeleton = output_skeleton_path;
	last_file_graph = output_graph_path;
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "skeletonize_server");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  voxblox::SkeletonizerNode skeletonizer(nh, nh_private);
  skeletonizer.init();

  ros::spin();
  return 0;
}
