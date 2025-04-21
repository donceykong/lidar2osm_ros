// export OMP_NUM_THREADS=1

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <random>     // For random sampling
#include <algorithm>  // For std::shuffle, std::sample
#include <tuple>      // for filterSemanticAssociations method

// Eigen :D
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CLIPPER
#include <clipper/clipper.h>
#include <clipper/utils.h>

// Lidar2OSM
#include "lidar2osm_ros/kitti.hpp"

class MapRecieverNode : public rclcpp::Node {
public:
  // Constructor
  MapRecieverNode(const std::string& robot_name) 
    : Node(robot_name + "_map_reciever_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
      // Create list of all other robot names
      std::array<std::string, 4> allRobotNames = {"robot1", "robot2", "robot3", "robot4"};
      std::array<std::string, 3> robotTeamNames;
      std::copy_if(allRobotNames.begin(), allRobotNames.end(), robotTeamNames.begin(),
                             [&robot_name](const std::string& name) { return name != robot_name; });

      // node params
      this->declare_parameter<std::string>("robot_name", robot_name);
      this->declare_parameter<std::string>("ego_frame", robot_name + "_base_link");
      this->declare_parameter<std::string>("teammate_b_name", robotTeamNames[0]);
      this->declare_parameter<std::string>("teammate_b_frame", robotTeamNames[0] + "_base_link");
      this->declare_parameter<double>("min_dist_threshold", 1.0);
      this->declare_parameter<double>("eff_comms_dist_threshold", 25.0);
      
      this->get_parameter("ego_frame", ego_frame_);
      this->get_parameter("robot_name", robot_name_);
      this->get_parameter("teammate_b_name", teammate_b_name_);
      this->get_parameter("teammate_b_frame", teammate_b_frame_);
      this->get_parameter("min_dist_threshold", min_dist_threshold_);
      this->get_parameter("eff_comms_dist_threshold", eff_comms_dist_threshold_);

      // // Global map subscriber
      std::string globalMapTopic = "/" + teammate_b_name_ + "/global_pointcloud";
      // std::string globalMapTopic = "/" + teammate_b_name_ + "/ouster/semantic_points";
      // global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //   globalMapTopic, rclcpp::SensorDataQoS(),
      //   std::bind(&MapRecieverNode::checkPeerDistance, this, std::placeholders::_1));

      // Global map sub
      std::string globalEgoMapTopic = "/" + robot_name_ + "/global_pointcloud";
      // std::string globalEgoMapTopic = "/" + robot_name_ + "/ouster/semantic_points";
      ego_global_map_.subscribe(this, globalEgoMapTopic);
      peer_global_map_.subscribe(this, globalMapTopic);
        
      sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(10), ego_global_map_, peer_global_map_);
      sync_->registerCallback(
        std::bind(&MapRecieverNode::checkPeerDistance, this, std::placeholders::_1, std::placeholders::_2));
        
      // Publisher for transformed peer cloud
      std::string peerMapTopic = "/" + robot_name + "/tf_peer_cloud";
      peer_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(peerMapTopic, 10);

      // Publisher for transformed ego cloud
      std::string egoMapTopic = "/" + robot_name + "/tf_ego_cloud";
      ego_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(egoMapTopic, 10);
  }
  
  void checkPeerDistance(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ego_global_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& peer_global_map) {
    try {
      // lookup peer and current frames
      geometry_msgs::msg::TransformStamped tf_a = tf_buffer_.lookupTransform("world", ego_frame_, tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped tf_b = tf_buffer_.lookupTransform("world", teammate_b_frame_, tf2::TimePointZero);

      // Compute Euclidean distance
      double dx = tf_a.transform.translation.x - tf_b.transform.translation.x;
      double dy = tf_a.transform.translation.y - tf_b.transform.translation.y;
      double dz = tf_a.transform.translation.z - tf_b.transform.translation.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      
      bool within_dist = false;

      // Only merge maps if they are within some distance (simulate comms constraints)
      if (distance <= eff_comms_dist_threshold_ && distance >= min_dist_threshold_) { 
        within_dist = true;
        eff_comms_dist_threshold_ = distance;
      }
      transformCloud(ego_global_map, peer_global_map, within_dist);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform from world to %s: %s", teammate_b_frame_.c_str(), ex.what());
      return;
    }
  }
  
  void transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ego_global_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& peer_global_map,
    bool within_dist) {

      geometry_msgs::msg::TransformStamped tfs_peer_to_map2;
      geometry_msgs::msg::TransformStamped tfs_ego_to_peer;   // True p2p tf
      geometry_msgs::msg::TransformStamped tfs_map1_to_ego;
      try {
          // Replace TimePointZero with //msg->header.stamp, rclcpp::Duration::from_seconds(0.1)); ??
          tfs_peer_to_map2 = tf_buffer_.lookupTransform(teammate_b_frame_, "world", tf2::TimePointZero);
          tfs_ego_to_peer = tf_buffer_.lookupTransform(ego_frame_, teammate_b_frame_, tf2::TimePointZero);
          tfs_map1_to_ego = tf_buffer_.lookupTransform("world", ego_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Could not get transform from world to %s: %s", teammate_b_frame_.c_str(), ex.what());
          return;
      }

      Eigen::Affine3d map1_to_ego = tf2::transformToEigen(tfs_map1_to_ego.transform);   //
      Eigen::Affine3d ego_to_peer = tf2::transformToEigen(tfs_ego_to_peer.transform);   //
      Eigen::Affine3d peer_to_map2 = tf2::transformToEigen(tfs_peer_to_map2.transform); //

      // // Convert PointCloud2 rosmsg to PCL PointCloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_peer_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
      pcl::fromROSMsg(*peer_global_map, *tf_peer_cloud);

      for (auto &point : tf_peer_cloud->points) {
          Eigen::Vector3d points_map2(point.x, point.y, point.z);
          Eigen::Vector3d transformed_p = peer_to_map2 * points_map2;

          point.x = transformed_p.x();
          point.y = transformed_p.y();
          point.z = transformed_p.z();
      }
      // publishPeerPointCloud(tf_peer_cloud);

      // // Convert PointCloud2 rosmsg to PCL PointCloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_ego_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*ego_global_map, *tf_ego_cloud);
      
      for (auto &point : tf_ego_cloud->points) {
          Eigen::Vector3d points_map1(point.x, point.y, point.z);
          Eigen::Vector3d transformed_p = map1_to_ego.inverse() * points_map1;

          point.x = transformed_p.x();
          point.y = transformed_p.y();
          point.z = transformed_p.z();
      }
      publishEgoPointCloud(tf_ego_cloud);

      if (within_dist) {
        // Now merge the maps
        Eigen::Affine3d ego_to_peer_est = mergeMaps(tf_peer_cloud, tf_ego_cloud);

        RCLCPP_INFO_STREAM(this->get_logger(), "\n\nego_to_peer_true:\n" << ego_to_peer.matrix());
        RCLCPP_INFO_STREAM(this->get_logger(), "\n\nego_to_peer_est:\n" << ego_to_peer_est.matrix());


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_peer_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*peer_global_map, *tf_peer_cloud2);
        for (auto &point : tf_peer_cloud2->points) {
          Eigen::Vector3d points_map2(point.x, point.y, point.z);
          Eigen::Vector3d transformed_p = map1_to_ego * ego_to_peer * peer_to_map2 * points_map2;

          point.x = transformed_p.x();
          point.y = transformed_p.y();
          point.z = transformed_p.z();
        }

        // pub TF'd peer cloud for viz purposes
        publishPeerPointCloud(tf_peer_cloud2);
      }
  }

  Eigen::Affine3d mergeMaps(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_ego_cloud, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_peer_cloud) {
    
    // Dont forget to change this if the voxel size changes
    int max_size_A = 10000;
    double map_voxel_size = 1.0;
    double eps = map_voxel_size * 5;

    // Convert ego and peer cloud to EigenXf mats
    Eigen::MatrixXd target_cloud_eigenXf = globalMapToEigen(tf_ego_cloud);
    Eigen::MatrixXd source_cloud_eigenXf = globalMapToEigen(tf_peer_cloud);

    // instantiate the invariant function that will be used to score associations
    clipper::invariants::EuclideanDistance::Params iparams;
    iparams.epsilon = eps;
    iparams.sigma = 0.5 * iparams.epsilon;
    clipper::invariants::EuclideanDistancePtr invariant =
              std::make_shared<clipper::invariants::EuclideanDistance>(iparams);

    // set up CLIPPER rounding parameters
    clipper::Params params;
    params.rounding = clipper::Params::Rounding::DSD_HEU;

    // instantiate clipper object
    clipper::CLIPPER clipper(invariant, params);

    // create A2A associations
    int target_points_len = target_cloud_eigenXf.rows();
    int source_points_len = source_cloud_eigenXf.rows();
    Eigen::Matrix<int, Eigen::Dynamic, 2> A_all_to_all = get_a2a_assoc_matrix(target_points_len, source_points_len);
    
    // Filter associations based on ego dist of points
    RCLCPP_INFO(this->get_logger(), "Filtering A: comms distance");
    Eigen::MatrixXd target_cloud_points_eigenXf = target_cloud_eigenXf.leftCols(3);
    Eigen::MatrixXd source_cloud_points_eigenXf = source_cloud_eigenXf.leftCols(3);
    Eigen::Matrix<int, Eigen::Dynamic, 2> A_ego_filtered = filter_by_ego_distance(
      target_cloud_points_eigenXf, source_cloud_points_eigenXf, A_all_to_all, eff_comms_dist_threshold_);
    
    // Filter associations based on semantic of points
    RCLCPP_INFO(this->get_logger(), "Filtering A: semantics");
    Eigen::Matrix<int, Eigen::Dynamic, 2>  A_sem_filtered;
    std::vector<int32_t> corr_filtered_labels;
    std::vector<int32_t> target_labels(target_cloud_eigenXf.rows());
    for (int i = 0; i < target_cloud_eigenXf.rows(); ++i) {
        target_labels[i] = static_cast<int32_t>(target_cloud_eigenXf(i, 3));
    }
    std::vector<int32_t> source_labels(source_cloud_eigenXf.rows());
    for (int i = 0; i < source_cloud_eigenXf.rows(); ++i) {
        source_labels[i] = static_cast<int32_t>(source_cloud_eigenXf(i, 3));
    }
    std::tie(A_sem_filtered, corr_filtered_labels) = filterSemanticAssociations(
      target_labels, source_labels, A_ego_filtered);
    
    // Filter associations based on a maximum # of associations
    RCLCPP_INFO(this->get_logger(), "Filtering A: max # specified associations");
    Eigen::Matrix<int, Eigen::Dynamic, 2>  A_filtered;
    std::tie(A_filtered, corr_filtered_labels) = downsampleAssociationMatrix(
      A_sem_filtered, max_size_A, corr_filtered_labels);

    // Score using invariant above and solve for maximal clique
    RCLCPP_INFO(this->get_logger(), "SCORING NOW");
    clipper.scorePairwiseConsistency(target_cloud_points_eigenXf.transpose(), target_cloud_points_eigenXf.transpose(), A_filtered);
    // clipper::Affinity M = clipper.getAffinityMatrix();

    RCLCPP_INFO(this->get_logger(), "SOLVING NOW");
    // clipper.solve();
    clipper.solveAsMaximumClique();

    // Retrieve selected inliers
    Eigen::Matrix<int, Eigen::Dynamic, 2> Ainliers = clipper.getSelectedAssociations();

    int Ainliers_len = Ainliers.rows();
    RCLCPP_INFO(
      this->get_logger(), 
      "Ainliers_len: %d", 
      Ainliers_len);

    // Compute peer2peer TF estimate
    Eigen::Matrix4d tf_est = computeTransformationFromInliers(target_cloud_points_eigenXf, source_cloud_points_eigenXf, Ainliers);
    Eigen::Affine3d tf_est_affine(tf_est);

    return tf_est_affine;
  }

  // All-to-All Association matrix
  Eigen::Matrix<int, Eigen::Dynamic, 2> get_a2a_assoc_matrix(int N1, int N2) {
    Eigen::Matrix<int, Eigen::Dynamic, 2> assoc_matrix(N1 * N2, 2);
    
    int i = 0;
    for (int n1 = 0; n1 < N1; ++n1) {
        for (int n2 = 0; n2 < N2; ++n2) {
            assoc_matrix(i, 0) = n1;
            assoc_matrix(i, 1) = n2;
            ++i;
        }
    }

    return assoc_matrix;
  }

  // Ego dist filterer
  Eigen::Matrix<int, Eigen::Dynamic, 2> filter_by_ego_distance(
    const Eigen::MatrixXd& pc1, 
    const Eigen::MatrixXd& pc2, 
    const Eigen::Matrix<int, Eigen::Dynamic, 2>& A, 
    double max_ego_dist) 
  {
    std::vector<Eigen::Vector2i> valid_rows;

    for (int i = 0; i < A.rows(); ++i) {
        int idx1 = A(i, 0);
        int idx2 = A(i, 1);

        if (idx1 >= pc1.rows() || idx2 >= pc2.rows()) {
            continue;  // safety check
        }

        float dist1 = pc1.row(idx1).norm();
        float dist2 = pc2.row(idx2).norm();

        float relative_ego_dist = std::abs(dist1 - dist2);

        if (relative_ego_dist < max_ego_dist) {
            valid_rows.emplace_back(idx1, idx2);  // <-- correctly push a Vector2i
        }
    }

    // Convert to Eigen matrix
    Eigen::Matrix<int, Eigen::Dynamic, 2> Anew(valid_rows.size(), 2);
    for (size_t i = 0; i < valid_rows.size(); ++i) {
        Anew(i, 0) = valid_rows[i][0];
        Anew(i, 1) = valid_rows[i][1];
    }

    return Anew;
  }

  // Semantic Filter
  std::tuple<Eigen::Matrix<int, Eigen::Dynamic, 2>, std::vector<int32_t>> filterSemanticAssociations(
    const std::vector<int32_t>& labels1,
    const std::vector<int32_t>& labels2,
    const Eigen::Matrix<int, Eigen::Dynamic, 2>& A)
  {
    std::vector<Eigen::RowVector2i> filtered_rows;
    std::vector<int32_t> filteredLabels;

    for (int i = 0; i < A.rows(); ++i) {
        // Fetch indices from the association matrix
        int index1 = A(i, 0);
        int index2 = A(i, 1);

        // Get the labels from the respective indices
        int32_t label1 = labels1[index1];
        int32_t label2 = labels2[index2];

        // Verify semantic labels are consistent
        if (label1 == label2 && label1 != -1) {
            // Add the association and the label
            filtered_rows.emplace_back(Eigen::RowVector2i(index1, index2));
            filteredLabels.push_back(label1);
        }
    }

    // Convert the filtered rows to an Eigen matrix
    Eigen::Matrix<int, Eigen::Dynamic, 2> filteredA(filtered_rows.size(), 2);
    for (size_t i = 0; i < filtered_rows.size(); ++i) {
        filteredA.row(i) = filtered_rows[i];
    }

    return std::make_tuple(filteredA, filteredLabels);
  }

  // Filter based on set max # of associations
  std::tuple<Eigen::Matrix<int, Eigen::Dynamic, 2>, std::vector<int32_t>> downsampleAssociationMatrix(
    const Eigen::Matrix<int, Eigen::Dynamic, 2>& A, 
    int max_size_A, 
    const std::vector<int32_t>& corr_labels)
  {
    int N = A.rows();
    max_size_A = std::min(max_size_A, N);  // avoid overflow

    // Generate random unique indices
    std::vector<int> indices(N);
    std::iota(indices.begin(), indices.end(), 0);  // fill with 0..N-1

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    std::vector<int> rand_ds_A_idxs(indices.begin(), indices.begin() + max_size_A);

    // Downsample A
    Eigen::Matrix<int, Eigen::Dynamic, 2> A_ds(max_size_A, 2);
    for (int i = 0; i < max_size_A; ++i) {
        A_ds.row(i) = A.row(rand_ds_A_idxs[i]);
    }

    // Downsample labels if provided
    std::vector<int32_t> corr_labels_ds;
    if (!corr_labels.empty()) {
        corr_labels_ds.resize(max_size_A);
        for (int i = 0; i < max_size_A; ++i) {
            corr_labels_ds[i] = corr_labels[rand_ds_A_idxs[i]];
        }
    }

    return std::make_tuple(A_ds, corr_labels_ds);
  }

  Eigen::Matrix4d umeyamaAlignment(const Eigen::MatrixXd& target, const Eigen::MatrixXd& source) {
    assert(target.rows() == source.rows());
    assert(target.cols() == 3 && source.cols() == 3);

    int N = target.rows();

    // compute centroids
    Eigen::Vector3d target_mean = target.colwise().mean();
    Eigen::Vector3d source_mean = source.colwise().mean();

    // center points
    Eigen::MatrixXd target_centered = target.rowwise() - target_mean.transpose();
    Eigen::MatrixXd source_centered = source.rowwise() - source_mean.transpose();

    // compute covariance matrix
    Eigen::Matrix3d H = target_centered.transpose() * source_centered;

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // compute rotation
    Eigen::Matrix3d R = V * U.transpose();

    // ensure proper rotation (no reflection)
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // compute translation
    Eigen::Vector3d t = source_mean - R * target_mean;

    // construct transformation matrix
    Eigen::Matrix4d Tfmat = Eigen::Matrix4d::Identity();
    Tfmat.block<3,3>(0,0) = R;
    Tfmat.block<3,1>(0,3) = t;

    return Tfmat;
  }

  Eigen::Matrix4d computeTransformationFromInliers(
    const Eigen::MatrixXd& target_cloud,
    const Eigen::MatrixXd& source_cloud,
    const Eigen::Matrix<int, Eigen::Dynamic, 2>& corres) {

    int N = corres.rows();
    Eigen::MatrixXd target_corr(N, 3);
    Eigen::MatrixXd source_corr(N, 3);

    for (int i = 0; i < N; ++i) {
        int target_idx = corres(i, 0);
        int source_idx = corres(i, 1);

        target_corr.row(i) = target_cloud.row(target_idx);
        source_corr.row(i) = source_cloud.row(source_idx);
    }

    // align dem bad boys
    return umeyamaAlignment(target_corr, source_corr);
  }

  Eigen::MatrixXd globalMapToEigen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud) {
      size_t original_size = pcl_cloud->size();
      size_t target_size = std::min<size_t>(10000, original_size);  // Limit to 10,000

      // Generate indices and randomly shuffle
      std::vector<size_t> indices(original_size);
      std::iota(indices.begin(), indices.end(), 0);     // Fill with 0,1,2,...,original_size-1
      std::random_device rd;
      std::mt19937 g(rd());                             // Random seed
      std::shuffle(indices.begin(), indices.end(), g);

      // convert the PCL points to an Eigen Matrix (XYZ + Label)
      Eigen::MatrixXd eigen_cloud(target_size, 4);
      for (size_t i = 0; i < target_size; ++i) {
          size_t idx = indices[i];  // Pick a random index

          eigen_cloud(i, 0) = pcl_cloud->points[idx].x;
          eigen_cloud(i, 1) = pcl_cloud->points[idx].y;
          eigen_cloud(i, 2) = pcl_cloud->points[idx].z;

          // Extract RGB values
          uint32_t rgb_val = *reinterpret_cast<int*>(&pcl_cloud->points[idx].rgb);
          int r = (rgb_val >> 16) & 0x0000ff;
          int g = (rgb_val >> 8) & 0x0000ff;
          int b = (rgb_val)&0x0000ff;

          eigen_cloud(i, 3) = getLabelFromRGB(r, g, b);  // Assign label
      }

      // RCLCPP_INFO(
      //   this->get_logger(), 
      //   "Converted global map to Eigen matrix with %ld points (downsampled from %ld)", 
      //   target_size, original_size);
      return(eigen_cloud);
  }

private:
  // Publish the peer's semantic cloud after transforming and after merging
  void publishPeerPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pcl_cloud, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = "world";
    peer_pointcloud_pub_->publish(cloud_msg);
  }

  void publishEgoPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pcl_cloud, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = "world";
    ego_pointcloud_pub_->publish(cloud_msg);
  }

  // ROS 2 timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2 listener and buffer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  std::string robot_name_;
  std::string ego_frame_;
  std::string teammate_b_name_;
  std::string teammate_b_frame_;
  double min_dist_threshold_;
  double eff_comms_dist_threshold_;

  // PointCloud2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;

  // Async PC2 subscriber
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ego_global_map_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> peer_global_map_;
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

  // PC2 Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr peer_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ego_pointcloud_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
  
    if (argc < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("map_reciever_node"), "Usage: map_reciever_node <robot_name>");
      return 1;
    }
  
    std::string robot_name = argv[1];
    rclcpp::spin(std::make_shared<MapRecieverNode>(robot_name));
    rclcpp::shutdown();
    return 0;
  }