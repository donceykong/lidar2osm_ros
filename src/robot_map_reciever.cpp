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
      this->declare_parameter<std::string>("teammate_b_name", robotTeamNames[2]);
      this->declare_parameter<std::string>("teammate_b_frame", robotTeamNames[2] + "_base_link");
      this->declare_parameter<double>("min_dist_threshold", 1.0);
      this->declare_parameter<double>("eff_comms_dist_threshold", 15.0);
      this->declare_parameter<int>("max_points_local_cloud", 10000);
      this->declare_parameter<int>("max_points_global_cloud", 100000);
      this->declare_parameter<int>("max_associations", 5000);
      this->declare_parameter<double>("map_voxel_size", 1.0);

      this->get_parameter("ego_frame", ego_frame_);
      this->get_parameter("robot_name", robot_name_);
      this->get_parameter("teammate_b_name", teammate_b_name_);
      this->get_parameter("teammate_b_frame", teammate_b_frame_);
      this->get_parameter("min_dist_threshold", min_dist_threshold_);
      this->get_parameter("eff_comms_dist_threshold", eff_comms_dist_threshold_);
      this->get_parameter("max_points_local_cloud", max_points_local_cloud_);
      this->get_parameter("max_points_global_cloud", max_points_global_cloud_);
      this->get_parameter("max_associations", max_associations_);
      this->get_parameter("map_voxel_size", map_voxel_size_);
      
      // Global map subscribers
      std::string localEgoMapTopic = "/" + robot_name_ + "/local_map";
      std::string localPeerMapTopic = "/" + teammate_b_name_ + "/local_map";
      std::string globalEgoMapTopic = "/" + robot_name_ + "/global_map";
      std::string globalPeerMapTopic = "/" + teammate_b_name_ + "/global_map";
      ego_local_map_.subscribe(this, localEgoMapTopic);
      peer_local_map_.subscribe(this, localPeerMapTopic);
      ego_global_map_.subscribe(this, globalEgoMapTopic);
      peer_global_map_.subscribe(this, globalPeerMapTopic);

      sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(10), 
        ego_local_map_, 
        peer_local_map_, 
        ego_global_map_, 
        peer_global_map_);
    
      sync_->registerCallback(
        std::bind(&MapRecieverNode::checkPeerDistance, this,
                  std::placeholders::_1, std::placeholders::_2, 
                  std::placeholders::_3, std::placeholders::_4));
      
      // Publisher for transformed peer cloud
      std::string peerMapTopic = "/" + robot_name + "/tf_peer_cloud";
      peer_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(peerMapTopic, 10);

      // Publisher for transformed ego cloud
      std::string egoMapTopic = "/" + robot_name + "/tf_ego_cloud";
      ego_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(egoMapTopic, 10);
  }
  
  void checkPeerDistance(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ego_local_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& peer_local_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ego_global_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& peer_global_map) 
  {
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
        // eff_comms_dist_threshold_ = distance;
      }
      transformCloud(ego_local_map, peer_local_map, ego_global_map, peer_global_map, within_dist);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform from world to %s: %s", teammate_b_frame_.c_str(), ex.what());
      return;
    }
  }
  
  void transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ego_local_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& peer_local_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ego_global_map,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& peer_global_map,
    bool within_dist) 
  {
      Eigen::Affine3d map1_to_ego;  // TF from peer to peer's map (starting pose)
      Eigen::Affine3d ego_to_peer;  // True p2p tf
      Eigen::Affine3d peer_to_map2; // TF from ego map to ego pose
      try {
          // Replace TimePointZero with //msg->header.stamp, rclcpp::Duration::from_seconds(0.1)); ??
          geometry_msgs::msg::TransformStamped tfs_peer_to_map2 = tf_buffer_.lookupTransform(teammate_b_frame_, "world", tf2::TimePointZero);
          geometry_msgs::msg::TransformStamped tfs_ego_to_peer = tf_buffer_.lookupTransform(ego_frame_, teammate_b_frame_, tf2::TimePointZero);
          geometry_msgs::msg::TransformStamped tfs_map1_to_ego = tf_buffer_.lookupTransform("world", ego_frame_, tf2::TimePointZero);
  
          map1_to_ego = tf2::transformToEigen(tfs_map1_to_ego.transform);
          ego_to_peer = tf2::transformToEigen(tfs_ego_to_peer.transform);
          peer_to_map2 = tf2::transformToEigen(tfs_peer_to_map2.transform);
      } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Could not get transform from world to %s: %s", teammate_b_frame_.c_str(), ex.what());
          return;
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr peer_orig_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
      pcl::fromROSMsg(*peer_local_map, *peer_orig_cloud);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ego_orig_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*ego_local_map, *ego_orig_cloud);

      // Convert ego_orig_cloud and peer_orig_cloud to MatrixXd from beginning.
      // This will ensure the indices of tf'd are in tact.
      Eigen::MatrixXd target_cloud_orig = PCLToEigen(ego_orig_cloud, max_points_local_cloud_);
      Eigen::MatrixXd source_cloud_orig = PCLToEigen(peer_orig_cloud, max_points_local_cloud_);

      // Create two TF'd to origin PC matrices
      Eigen::MatrixXd tf_target_cloud = transformEigenCloud(target_cloud_orig, map1_to_ego.inverse());
      Eigen::MatrixXd tf_source_cloud = transformEigenCloud(source_cloud_orig, peer_to_map2);

      if (within_dist) {
        Eigen::Affine3d ego_to_peer_est = mergeMaps(target_cloud_orig, tf_target_cloud, source_cloud_orig, tf_source_cloud);

        RCLCPP_INFO_STREAM(this->get_logger(), "\n\nego_to_peer_true:\n" << ego_to_peer.matrix());
        RCLCPP_INFO_STREAM(this->get_logger(), "\n\nego_to_peer_est:\n" << ego_to_peer_est.matrix());

        // Visualize global peer cloud transformed
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr peer_orig_global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
        pcl::fromROSMsg(*peer_global_map, *peer_orig_global_cloud);
        Eigen::MatrixXd source_global_cloud_orig = PCLToEigen(peer_orig_global_cloud, max_points_global_cloud_);

        Eigen::Affine3d tf_to_world =  map1_to_ego * ego_to_peer_est * peer_to_map2;
        Eigen::MatrixXd source_cloud_est = transformEigenCloud(source_global_cloud_orig, tf_to_world);
        auto pcl_source_cloud_est= eigenToPCL(source_cloud_est);
        publishPeerPointCloud(pcl_source_cloud_est);
      }
  }

  Eigen::Affine3d mergeMaps(
    Eigen::MatrixXd target_cloud_orig,
    Eigen::MatrixXd tf_target_cloud,
    Eigen::MatrixXd source_cloud_orig, 
    Eigen::MatrixXd tf_source_cloud) 
  {
    // Set epsilon according to input map's voxel leaf len
    // a multiplier of 5 seemed to work well for many tests
    double eps = map_voxel_size_ * 5;

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
    // RCLCPP_INFO(this->get_logger(), "Filtering A: special filtering");
    // clipper::Association A_all_to_all = get_spec_a2a_assoc_matrix(target_cloud_orig, source_cloud_orig, 2.0);
    RCLCPP_INFO(this->get_logger(), "Creating A2A Associations");
    int target_points_len = target_cloud_orig.rows();
    int source_points_len = source_cloud_orig.rows();
    clipper::Association A_all_to_all = get_a2a_assoc_matrix(target_points_len, source_points_len);
    
    Eigen::MatrixXd tf_target_cloud_points = tf_target_cloud.leftCols(3);
    Eigen::MatrixXd tf_source_cloud_points = tf_source_cloud.leftCols(3);

    // Filter associations based on ego dist of points
    RCLCPP_INFO(this->get_logger(), "Filtering A: comms distance");
    clipper::Association A_ego_filtered = filter_by_ego_distance(
      tf_target_cloud_points, tf_source_cloud_points, A_all_to_all);

    // Filter associations based on semantic of points
    RCLCPP_INFO(this->get_logger(), "Filtering A: semantics");
    clipper::Association  A_sem_filtered;
    std::vector<int> corr_filtered_labels;
    std::vector<int>target_labels(tf_target_cloud.rows());
    for (int i = 0; i < tf_target_cloud.rows(); ++i) {
        target_labels[i] = static_cast<int32_t>(tf_target_cloud(i, 3));
    }
    std::vector<int>  source_labels(tf_source_cloud.rows());
    for (int i = 0; i < tf_source_cloud.rows(); ++i) {
        source_labels[i] = static_cast<int32_t>(tf_source_cloud(i, 3));
    }
    std::tie(A_sem_filtered, corr_filtered_labels) = filterSemanticAssociations(
      target_labels, source_labels, A_ego_filtered);
    
    // Filter associations based on a maximum # of associations
    RCLCPP_INFO(this->get_logger(), "Filtering A: max # specified associations");
    clipper::Association  A_filtered;
    std::tie(A_filtered, corr_filtered_labels) = downsampleAssociationMatrix(
      A_sem_filtered, corr_filtered_labels);

    // Score using invariant above and solve for maximal clique
    RCLCPP_INFO(this->get_logger(), "SCORING NOW");
    clipper.scorePairwiseConsistency(tf_target_cloud_points.transpose(), tf_source_cloud_points.transpose(), A_filtered);
    // clipper::Affinity M = clipper.getAffinityMatrix();

    RCLCPP_INFO(this->get_logger(), "SOLVING NOW");
    // clipper.solve();
    clipper.solveAsMaximumClique();

    // Retrieve selected inliers
    clipper::Association Ainliers = clipper.getSelectedAssociations();
    RCLCPP_INFO(this->get_logger(), "Ainliers_len: %d", Ainliers.rows());

    // Compute peer2peer TF estimate
    RCLCPP_INFO(this->get_logger(), "COMPUTING TF");
    Eigen::Affine3d tf_est_affine = computeTransformationFromInliers(tf_target_cloud_points, tf_source_cloud_points, Ainliers);

    return tf_est_affine;
  }

private:
  // Return pairs of indices where point pairs from pc1 and pc2 are within max_dist
  clipper::Association get_spec_a2a_assoc_matrix(
    const Eigen::MatrixXd& pc1,
    const Eigen::MatrixXd& pc2,
    double max_dist)
  {
    std::vector<Eigen::Vector2i> associations;

    int N1 = pc1.rows();
    int N2 = pc2.rows();

    for (int i = 0; i < N1; ++i) {
      for (int j = 0; j < N2; ++j) {
        double dist = (pc1.row(i) - pc2.row(j)).norm();
        if (dist <= max_dist) {
          associations.emplace_back(i, j);
        }
      }
    }

    clipper::Association assoc_matrix(associations.size(), 2);
    for (int k = 0; k < associations.size(); ++k) {
      assoc_matrix.row(k) = associations[k];
    }

    return assoc_matrix;
  }


  // All-to-All Association matrix
  clipper::Association get_a2a_assoc_matrix(
    int N1, 
    int N2) 
  {
    clipper::Association assoc_matrix(N1 * N2, 2);
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
  clipper::Association filter_by_ego_distance(
    const Eigen::MatrixXd& pc1,
    const Eigen::MatrixXd& pc2,
    const clipper::Association& A) 
  {
    std::vector<Eigen::Vector2i> valid_rows;

    for (int i = 0; i < A.rows(); ++i) {
        int idx1 = A(i, 0);
        int idx2 = A(i, 1);

        float dist1 = pc1.row(idx1).norm();
        float dist2 = pc2.row(idx2).norm();

        float relative_ego_dist = std::abs(dist1 - dist2);

        if (relative_ego_dist < eff_comms_dist_threshold_) {
            valid_rows.emplace_back(idx1, idx2);
        }
    }

    // Create new association mat
    clipper::Association Anew(valid_rows.size(), 2);
    for (size_t i = 0; i < valid_rows.size(); ++i) {
        Anew(i, 0) = valid_rows[i][0];
        Anew(i, 1) = valid_rows[i][1];
    }

    return Anew;
  }

  // Semantic Filter
  std::tuple<clipper::Association, std::vector<int> > filterSemanticAssociations(
    const std::vector<int> & labels1,
    const std::vector<int> & labels2,
    const clipper::Association& A)
  {
    std::vector<Eigen::RowVector2i> filtered_rows;
    std::vector<int>  filteredLabels;

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
    clipper::Association filteredA(filtered_rows.size(), 2);
    for (size_t i = 0; i < filtered_rows.size(); ++i) {
        filteredA.row(i) = filtered_rows[i];
    }

    return std::make_tuple(filteredA, filteredLabels);
  }

  // Filter based on set max # of associations
  std::tuple<clipper::Association, std::vector<int> > downsampleAssociationMatrix(
    const clipper::Association& A,
    const std::vector<int> & corr_labels)
  {
    int N = A.rows();
    int max_size_A = std::min(max_associations_, N);  // avoid overflow

    // Generate random unique indices
    std::vector<int> indices(N);
    std::iota(indices.begin(), indices.end(), 0);  // fill with 0..N-1

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    // Downsample A
    std::vector<int> rand_ds_A_idxs(indices.begin(), indices.begin() + max_size_A);
    clipper::Association A_ds(max_size_A, 2);
    for (int i = 0; i < max_size_A; ++i) {
        A_ds.row(i) = A.row(rand_ds_A_idxs[i]);
    }

    // Downsample labels if provided
    std::vector<int>  corr_labels_ds;
    if (!corr_labels.empty()) {
        corr_labels_ds.resize(max_size_A);
        for (int i = 0; i < max_size_A; ++i) {
            corr_labels_ds[i] = corr_labels[rand_ds_A_idxs[i]];
        }
    }

    return std::make_tuple(A_ds, corr_labels_ds);
  }

  Eigen::Matrix4d umeyamaAlignment(
    const Eigen::MatrixXd& target, 
    const Eigen::MatrixXd& source) 
  {
    assert(target.rows() == source.rows());

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

  Eigen::Affine3d computeTransformationFromInliers(
    const Eigen::MatrixXd& target_cloud,
    const Eigen::MatrixXd& source_cloud,
    const clipper::Association& corres) 
  {

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
    Eigen::Matrix4d tf_est = umeyamaAlignment(source_corr, target_corr);
    Eigen::Affine3d tf_est_affine(tf_est);

    return tf_est_affine;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr eigenToPCL(
    const Eigen::MatrixXd& eigen_cloud)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cloud->points.reserve(eigen_cloud.rows());

    for (int i = 0; i < eigen_cloud.rows(); ++i) {
        pcl::PointXYZRGB pt;
        pt.x = eigen_cloud(i, 0);
        pt.y = eigen_cloud(i, 1);
        pt.z = eigen_cloud(i, 2);

        int label = static_cast<int>(eigen_cloud(i, 3));
        std::vector<int> rgb = getRGBFromLabel(label);

        if (rgb[0] >= 0) {  // Check for valid label
            uint32_t rgb_packed = (static_cast<uint32_t>(rgb[0]) << 16 |
                                  static_cast<uint32_t>(rgb[1]) << 8 |
                                  static_cast<uint32_t>(rgb[2]));
            pt.rgb = *reinterpret_cast<float*>(&rgb_packed);
        } else {
            pt.r = pt.g = pt.b = 0;  // fallback to black
        }

        pcl_cloud->points.push_back(pt);
    }

    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;

    return pcl_cloud;
  }

  Eigen::MatrixXd transformEigenCloud(
    const Eigen::MatrixXd& input_cloud,
    const Eigen::Affine3d& tf)
  {
    Eigen::MatrixXd output_cloud = input_cloud;

    for (int i = 0; i < output_cloud.rows(); ++i) {
        Eigen::Vector3d pt(output_cloud(i, 0), output_cloud(i, 1), output_cloud(i, 2));
        Eigen::Vector3d pt_tf = tf * pt;

        output_cloud(i, 0) = pt_tf.x();
        output_cloud(i, 1) = pt_tf.y();
        output_cloud(i, 2) = pt_tf.z();
    }

    return output_cloud;
  }

  Eigen::MatrixXd PCLToEigen(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
    int max_num_points) 
  {
    size_t original_size = pcl_cloud->size();
    size_t target_size = std::min<size_t>(max_num_points, original_size);  // Limit to max_num_points

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
        int b = (rgb_val) & 0x0000ff;

        eigen_cloud(i, 3) = getLabelFromRGB(r, g, b);  // Assign label
    }

    // RCLCPP_INFO(
    //   this->get_logger(), 
    //   "Converted global map to Eigen matrix with %ld points (downsampled from %ld)", 
    //   target_size, original_size);
    return(eigen_cloud);
  }

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

  // Limits for random sampling
  int max_points_local_cloud_;
  int max_points_global_cloud_;
  int max_associations_;
  double map_voxel_size_;

  // Async PC2 subscriber
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ego_local_map_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> peer_local_map_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ego_global_map_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> peer_global_map_;
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

  // PC2 Publishers
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