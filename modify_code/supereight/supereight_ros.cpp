// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/supereight_ros.hpp"

#include <cstring>
#include <functional>
#include <thread>

#include <lodepng.h>

#include <eigen_conversions/eigen_msg.h>

#include <se/config.h>

#include "supereight_ros/utilities.hpp"



namespace se {

SupereightNode::SupereightNode(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_(0),
      frame_id_("map") {

  readConfig(nh_private);

  init_position_octree_ = supereight_config_.initial_pos_factor.cwiseProduct(supereight_config_.volume_size);
  computation_size_ = node_config_.input_res / supereight_config_.compute_size_ratio;

  // Allocate input image buffers.
  const size_t input_num_pixels = node_config_.input_res.prod();
  input_depth_ = std::unique_ptr<uint16_t>(new uint16_t[input_num_pixels]);
  if (node_config_.enable_rgb) {
    input_rgb_ = std::unique_ptr<uint8_t>(new uint8_t[3 * input_num_pixels]);
  }

  // Allocate rendered image buffers.
  if (node_config_.enable_rendering) {
    const size_t render_num_pixels = computation_size_.prod();
    depth_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    if (node_config_.enable_rgb) {
      rgba_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    }
    if (node_config_.enable_tracking) {
      track_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    }
    volume_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
  }

  // Initialize the supereight pipeline.
  pipeline_ = std::shared_ptr<DenseSLAMSystem>(new DenseSLAMSystem(
      computation_size_,
      Eigen::Vector3i::Constant(supereight_config_.volume_resolution.x()),
      Eigen::Vector3f::Constant(supereight_config_.volume_size.x()),
      init_position_octree_,
      supereight_config_.pyramid,
      supereight_config_));

  // Initialize the timings.
  timings_.resize(8);
  timing_labels_ = {"Message preprocessing",
                    "Preprocessing",
                    "Tracking",
                    "Integration",
                    "Raycasting",
                    "Rendering",
                    "Visualization"};

  // Allocate message circular buffers.
  if (node_config_.enable_tracking) {
    pose_buffer_.set_capacity(node_config_.pose_buffer_size);
    //pose_buffer_.set_capacity(0);
    objpose_buffer_.set_capacity(node_config_.pose_buffer_size);
  } else {
    pose_buffer_.set_capacity(node_config_.pose_buffer_size);
  }
  depth_buffer_.set_capacity(node_config_.depth_buffer_size);
  if (node_config_.enable_rgb) {
    rgb_buffer_.set_capacity(node_config_.rgb_buffer_size);
  } else {
    rgb_buffer_.set_capacity(0);
  }

  //number_of_different_ = 0;
  setupRos();

  ROS_INFO("Initialization finished");
  
  //ros::Duration(1.0).sleep();

}



void SupereightNode::runPipelineOnce() {
  // runPipelineOnce() should only be run by a single thread at a time. Return
  // if the lock can't be acquired (another thread is already running).
  std::unique_lock<std::mutex> fusion_lock (fusion_mutex_, std::defer_lock_t());
  if (!fusion_lock.try_lock()) {
    return;
  }

  timings_[0] = std::chrono::steady_clock::now();



  // Message association
  // Depth
  sensor_msgs::ImageConstPtr current_depth_msg;
  double depth_timestamp;
  long depth_timestamp_nsec; 
  { // Block to reduce the scope of depth_lock.
    const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
    if (depth_buffer_.empty()) {
      ROS_DEBUG("Aborted fusion: depth buffer empty");
      return;
    } else {
      current_depth_msg = depth_buffer_.front();
      depth_timestamp = ros::Time(current_depth_msg->header.stamp).toSec();
      depth_timestamp_nsec = ros::Time(current_depth_msg->header.stamp).toNSec();
    }
  }

  // RGB
  sensor_msgs::ImageConstPtr current_rgb_msg;
  if (node_config_.enable_rgb) {
    const std::lock_guard<std::mutex> rgb_lock (rgb_buffer_mutex_);
    if (rgb_buffer_.empty()) {
      ROS_DEBUG("Aborted fusion: RGB buffer empty");
      return;
    } else {
      const bool found = get_closest_image(rgb_buffer_, depth_timestamp,
          node_config_.max_timestamp_diff, current_rgb_msg);
      if (!found) {
        ROS_DEBUG("Aborted fusion: could not find matching RGB");
        return;
      }
    }
  }

  // Pose
  Eigen::Matrix4f external_pose;
  if (!node_config_.enable_tracking) {
    const std::lock_guard<std::mutex> pose_lock (pose_buffer_mutex_);
    if (pose_buffer_.empty()) {
      // Clear the depth and RGB buffers if no poses have arrived yet. These
      // images will never be associated to poses.
      depth_buffer_.clear();
      rgb_buffer_.clear(); // OK to call even when RGB images are not used
      ROS_DEBUG("Aborted fusion: pose buffer empty");
      return;
    } else {
      // Find the two closest poses and interpolate to the depth timestamp.
      geometry_msgs::TransformStamped prev_pose;
      geometry_msgs::TransformStamped next_pose;


      // only integradted the samse time stamp
      const InterpResult result = get_surrounding_poses(
            pose_buffer_, depth_timestamp, prev_pose, next_pose);

      long time1, time2;
      time1 = ros::Time(prev_pose.header.stamp).toNSec();
      time2 = ros::Time(next_pose.header.stamp).toNSec();
	  
	  // Code: for make the sync the pose and image timestamp
      // if( depth_timestamp_nsec != time1 && time1 != time2)
      // {
      //   std::cout<< depth_timestamp<< "," << time1 << "," << time2 << std::endl;
      //   different1 = depth_timestamp;
      //   different2 = time1;
      //   different3 = time2;
      //   return;
      // }

      if (result == InterpResult::query_smaller) {
        // Remove the depth image, it will never be matched to poses.
        const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
        depth_buffer_.pop_front();
        ROS_DEBUG("Aborted fusion: query smaller than all poses");
        return;
      } else if (result == InterpResult::query_greater) {
        // Remove the first poses, they will never be matched to depth images.
        pose_buffer_.erase_begin(pose_buffer_.size() - 1);
        ROS_DEBUG("Aborted fusion: query greater than all poses");
        return;
      }

      // Interpolate to associate a pose to the depth image.
      external_pose = interpolate_pose(prev_pose, next_pose, depth_timestamp);

      if(time1 != time2 && depth_timestamp!= time1)
          number_of_different_ += 1;

      // std::cout <<"Check the Pose timestamp:"
      //           <<"\ndepth:"<< depth_timestamp_nsec
      //           <<"\nprev_pose: " << time1
      //           <<"\next_pose:" << time2
      //           <<"\nNumber of differnet timestamp:" << number_of_different_
      //           //<<"/n"<< different1 <<","<< different2 << ","<< different3
      // <<std::endl;
    }
  }
  // Code: when turn on the tracking , make a prediction by the predict model
  // else{
  //   //prediction:
  //   std::cout<< "prediction in tracking" << std::endl;

  //   const std::lock_guard<std::mutex> pose_lock (pose_buffer_mutex_);
  //   if (pose_buffer_.empty()) {
  //     depth_buffer_.clear();
  //     rgb_buffer_.clear(); // OK to call even when RGB images are not used
  //     std::cout<< "Aborted fusion: pose buffer empty" << std::endl;
  //     ROS_DEBUG("Aborted fusion: pose buffer empty");
  //     return;
  //   } else {
  //     // Find the two closest poses and interpolate to the depth timestamp.
  //     geometry_msgs::TransformStamped prev_pose;
  //     geometry_msgs::TransformStamped next_pose;

  //     // only integradted the samse time stamp
  //     const InterpResult result = get_surrounding_poses(
  //           pose_buffer_, depth_timestamp, prev_pose, next_pose);

  //     if (result == InterpResult::query_smaller) {
  //       // Remove the depth image, it will never be matched to poses.
  //       const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
  //       depth_buffer_.pop_front();
  //       std::cout<< "Aborted fusion: query smaller than all poses" << std::endl;
  //       ROS_DEBUG("Aborted fusion: query smaller than all poses");
  //       return;
  //     } else if (result == InterpResult::query_greater) {
  //       // Remove the first poses, they will never be matched to depth images.
  //       pose_buffer_.erase_begin(pose_buffer_.size() - 1);
  //       std::cout<< "Aborted fusion: query greater than all poses" << std::endl;
  //       ROS_DEBUG("Aborted fusion: query greater than all poses");
  //       return;
  //     }

  //     // Interpolate to associate a pose to the depth image.
  //     external_pose = interpolate_pose(prev_pose, next_pose, depth_timestamp);
  //   }


  // }

  // The currect depth image is going to be integrated, remove it from the
  // buffer to avoid integrating it again.
  { // Block to reduce the scope of depth_lock.
    const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
    depth_buffer_.pop_front();
  }
  // Copy the depth and RGB images into the buffers used by supereight.
  to_supereight_depth(*current_depth_msg, input_depth_.get());
  if (node_config_.enable_rgb) {
    to_supereight_RGB(*current_rgb_msg, input_rgb_.get());
  }
  timings_[1] = std::chrono::steady_clock::now();



  // Preprocessing
  pipeline_->preprocessDepth(input_depth_.get(), node_config_.input_res,
      supereight_config_.bilateral_filter);
  if (node_config_.enable_rgb) {
    pipeline_->preprocessColor(input_rgb_.get(), node_config_.input_res);
  }
  timings_[2] = std::chrono::steady_clock::now();



  // Tracking
  bool tracked = false;
  const Eigen::Vector4f camera = supereight_config_.camera / supereight_config_.compute_size_ratio;
  if (node_config_.enable_tracking) {
    if (frame_ % supereight_config_.tracking_rate == 0) {
		
	  //predict:
      //pipeline_->setPose(external_pose);
      tracked = pipeline_->track(camera, supereight_config_.icp_threshold);
      std::cout<< "Tracked result:" << tracked << std::endl;
    } else {
      tracked = false;
    }
  } else {
    pipeline_->setPose(external_pose);
    tracked = true;
  }

  const Eigen::Matrix4f tracked_pose = pipeline_->getPose();
  geometry_msgs::PoseStamped supereight_pose;
  supereight_pose.header = current_depth_msg->header;
  supereight_pose.header.frame_id = frame_id_;
  tf::pointEigenToMsg(tracked_pose.block<3, 1>(0, 3).cast<double>(), supereight_pose.pose.position);
  Eigen::Quaternionf q_rot(tracked_pose.block<3, 3>(0, 0));
  tf::quaternionEigenToMsg(q_rot.cast<double>(), supereight_pose.pose.orientation);
  // Publish pose estimated by supereight.
  supereight_pose_pub_.publish(supereight_pose);
  timings_[3] = std::chrono::steady_clock::now();



  // Integration
  // Integrate only if tracking was successful or it is one of the first 4
  // frames.
  bool integrated = false;
  if ((tracked && (frame_ % supereight_config_.integration_rate == 0)) || frame_ <= 3) {
    integrated = pipeline_->integrate(camera, supereight_config_.mu, frame_);
  } else {
    integrated = false;
  }
  timings_[4] = std::chrono::steady_clock::now();



  // Raycasting
  bool raycasted = false;
  if ((node_config_.enable_tracking || node_config_.enable_rendering) && frame_ > 2) {
    raycasted = pipeline_->raycast(camera, supereight_config_.mu);
  }
  timings_[5] = std::chrono::steady_clock::now();



  // Rendering
  if (node_config_.enable_rendering) {
    // Depth
    pipeline_->renderDepth((unsigned char*) depth_render_.get(), computation_size_);
    const sensor_msgs::Image depth_render_msg = RGBA_to_msg(depth_render_.get(),
        computation_size_, current_depth_msg->header);
    depth_render_pub_.publish(depth_render_msg);

    // RGB
    if (node_config_.enable_rgb) {
      pipeline_->renderRGBA((unsigned char*) rgba_render_.get(), computation_size_);
      const sensor_msgs::Image rgba_render_msg = RGBA_to_msg(rgba_render_.get(),
          computation_size_, current_depth_msg->header);
      rgba_render_pub_.publish(rgba_render_msg);
    }

    // Track
    if (node_config_.enable_tracking) {
      pipeline_->renderTrack((unsigned char*) track_render_.get(), computation_size_);
      const sensor_msgs::Image track_render_msg = RGBA_to_msg(track_render_.get(),
          computation_size_, current_depth_msg->header);
      track_render_pub_.publish(track_render_msg);
    }

    // Volume
    if (frame_ % supereight_config_.rendering_rate == 0) {
      pipeline_->renderVolume((unsigned char*) volume_render_.get(), computation_size_,
          camera, 0.75 * supereight_config_.mu);
      const sensor_msgs::Image volume_render_msg = RGBA_to_msg(volume_render_.get(),
          computation_size_, current_depth_msg->header);
      volume_render_pub_.publish(volume_render_msg);
    }
  }
  timings_[6] = std::chrono::steady_clock::now();



  // Visualization
  //if (std::is_same<FieldType, OFusion>::value) {
  //  visualizeMapOFusion(updated_blocks, frontier_blocks, frontier_blocks_map, occlusion_blocks);
  //} else if (std::is_same<FieldType, SDF>::value) {
  //  visualizeMapSDF(occupied_voxels, freed_voxels, updated_blocks);
  //}
  timings_[7] = std::chrono::steady_clock::now();



  ROS_INFO("-----------------------------------------");
  ROS_INFO("Frame %d", frame_);
  ROS_INFO("Tracked: %d   Integrated: %d   Raycasted: %d",
      tracked, integrated, raycasted);
  print_timings(timings_, timing_labels_);

  frame_++;
}



void SupereightNode::saveMap() {
  if (!supereight_config_.dump_volume_file.empty()) {
    pipeline_->dump_mesh(supereight_config_.dump_volume_file.c_str());
    ROS_INFO("Map saved in %s\n", supereight_config_.dump_volume_file.c_str());
  }
}



void SupereightNode::setupRos() {
  // Pose subscriber
  //if (!node_config_.enable_tracking) {
    if (node_config_.pose_topic_type == "geometry_msgs::PoseStamped") {
      pose_sub_ = nh_.subscribe("/pose", node_config_.pose_buffer_size,
          &SupereightNode::poseStampedCallback, this);

    } else if (node_config_.pose_topic_type == "geometry_msgs::TransformStamped") {
      pose_sub_ = nh_.subscribe("/pose", node_config_.pose_buffer_size,
          &SupereightNode::transformStampedCallback, this);

    } else {
      ROS_FATAL("Invalid pose topic type %s", node_config_.pose_topic_type.c_str());
      ROS_FATAL("Expected geometry_msgs::PoseStamped or geometry_msgs::TransformStamped");
      abort();
    }
  //}
  // Obj pose subscriber:
  if(node_config_.enable_tracking)
  {
    obj_sub_ = nh_.subscribe("/my_duck/pose", node_config_.pose_buffer_size, &SupereightNode::duckposeCallback, this);
  }

  // Depth subscriber
  depth_sub_ = nh_.subscribe("/camera/depth_image",
      node_config_.depth_buffer_size, &SupereightNode::depthCallback, this);
  // RGB subscriber
  if (node_config_.enable_rgb) {
    rgb_sub_ = nh_.subscribe("/camera/rgb_image", node_config_.rgb_buffer_size,
        &SupereightNode::RGBCallback, this);
  }

  // Publishers
  supereight_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/supereight/pose",
      node_config_.pose_buffer_size);
  // Render publishers
  if (node_config_.enable_rendering) {
    depth_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/depth_render", 30);
    if (node_config_.enable_rgb) {
      rgba_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/rgba_render", 30);
    }
    if (node_config_.enable_tracking) {
      track_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/track_render",30);
    }
    volume_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render",30);
  }

  // Visualization
  //map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_based_marker", 1);
  //block_based_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("block_based_marker", 1);
  //boundary_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("boundary_marker", 1);
  //frontier_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("frontier_marker", 1);
}



void SupereightNode::readConfig(const ros::NodeHandle& nh_private) {
  supereight_config_ = read_supereight_config(nh_private);
  print_supereight_config(supereight_config_);

  node_config_ = read_supereight_node_config(nh_private);
  print_supereight_node_config(node_config_);
};



void SupereightNode::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
  const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
  depth_buffer_.push_back(depth_msg);
  ROS_DEBUG("Depth image buffer: %lu/%lu", depth_buffer_.size(), depth_buffer_.capacity());
}



void SupereightNode::RGBCallback(const sensor_msgs::ImageConstPtr& rgb_msg) {
  const std::lock_guard<std::mutex> rgb_lock (rgb_buffer_mutex_);
  rgb_buffer_.push_back(rgb_msg);
  ROS_DEBUG("RGB image buffer:   %lu/%lu", rgb_buffer_.size(), rgb_buffer_.capacity());
}



void SupereightNode::poseStampedCallback(
    const geometry_msgs::PoseStamped::ConstPtr& T_WB_msg) {

  // Convert the message to an Eigen matrix.
  Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_WB;
  tf::quaternionMsgToEigen(T_WB_msg->pose.orientation, q_WB);
  T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
  Eigen::Vector3d t_WB;
  tf::pointMsgToEigen(T_WB_msg->pose.position, t_WB);
  T_WB.topRightCorner<3, 1>() = t_WB;

  // Call the generic pose callback.
  poseCallback(T_WB, T_WB_msg->header);
}


void SupereightNode::duckposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  // //conver the message to an Eigen matrix.
  Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_WB;
  tf::quaternionMsgToEigen(msg->pose.orientation, q_WB);
  T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
  Eigen::Vector3d t_WB;
  tf::pointMsgToEigen(msg->pose.position, t_WB);
  T_WB.topRightCorner<3, 1>() = t_WB;


  duckpose2Callback(T_WB, msg->header);  
}


void SupereightNode::transformStampedCallback(
    const geometry_msgs::TransformStamped::ConstPtr& T_WB_msg) {

  // Convert the message to an Eigen matrix.
  Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_WB;
  tf::quaternionMsgToEigen(T_WB_msg->transform.rotation, q_WB);
  T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
  Eigen::Vector3d t_WB;
  tf::vectorMsgToEigen(T_WB_msg->transform.translation, t_WB);
  T_WB.topRightCorner<3, 1>() = t_WB;

  // Call the generic pose callback.
  poseCallback(T_WB, T_WB_msg->header);
}


void SupereightNode::duckpose2Callback(const Eigen::Matrix4d&  T_WB,
                                       const std_msgs::Header& header)
{

  //const Eigen::Matrix4d T_WC = T_WB * supereight_config_.T_BC.cast<double>();
  const Eigen::Matrix4d T_WC = T_WB;
  geometry_msgs::TransformStamped T_WC_msg;

  T_WC_msg.header = header;
  T_WC_msg.header.frame_id = frame_id_;
  const Eigen::Quaterniond q_WC (T_WC.topLeftCorner<3, 3>());
  tf::quaternionEigenToMsg(q_WC, T_WC_msg.transform.rotation);
  const Eigen::Vector3d t_WC = T_WC.topRightCorner<3, 1>();
  tf::vectorEigenToMsg(t_WC, T_WC_msg.transform.translation);

   // Put it into the buffer.
  const std::lock_guard<std::mutex> pose_lock (objpose_buffer_mutex_);
  //std::cout<<"recieve the objpose:"<< ros::Time(msg->header.stamp).toSec() <<std::endl;
  objpose_buffer_.push_back(T_WC_msg);
  //std::cout<<"size:"<< objpose_buffer_.size() <<std::endl;


}

void SupereightNode::poseCallback(const Eigen::Matrix4d&  T_WB,
                                  const std_msgs::Header& header) {

  // Convert body pose to camera pose.
  const Eigen::Matrix4d T_WC = T_WB * supereight_config_.T_BC.cast<double>();

  // Create a ROS message from T_WC.
  geometry_msgs::TransformStamped T_WC_msg;
  T_WC_msg.header = header;
  T_WC_msg.header.frame_id = frame_id_;
  const Eigen::Quaterniond q_WC (T_WC.topLeftCorner<3, 3>());
  tf::quaternionEigenToMsg(q_WC, T_WC_msg.transform.rotation);
  const Eigen::Vector3d t_WC = T_WC.topRightCorner<3, 1>();
  tf::vectorEigenToMsg(t_WC, T_WC_msg.transform.translation);

  // Put it into the buffer.
  const std::lock_guard<std::mutex> pose_lock (pose_buffer_mutex_);
  pose_buffer_.push_back(T_WC_msg);
  ROS_DEBUG("Pose buffer:        %lu/%lu",
      pose_buffer_.size(), pose_buffer_.capacity());
}



void SupereightNode::saveTSDF()
{
  // std::shared_ptr<se::Octree<VoxelImpl::VoxelType> > octree_;
  // pipeline_->getMap(octree_);
  // node_iterator<T> node_it(*octree_);

  // auto surface_voxels = node_it.getSurfaceVoxels();

  std::cout<< "It's ok" << std::endl;

}


//void SupereightNode::visualizeMapOFusion(vec3i &updated_blocks,
//                                            vec3i &frontier_blocks,
//                                            map3i &frontier_blocks_map,
//                                            vec3i &occlusion_blocks) {
////void SupereightNode::visualizeMapOFusion(std::vector<Eigen::Vector3i>& updated_blocks,
////                                            std::vector<Eigen::Vector3i>& frontier_blocks) {
//  // publish every N-th frame
//  int N_frame_pub = 1;
//
//  // get supereight map
//  pipeline_->getMap(octree_);
//  node_iterator<T> node_it(*octree_);
//
//  // set with stored morton code
////  std::set<uint64_t> surface_voxel_set;
////  std::set<uint64_t> frontier_voxel_set;
////  std::set<uint64_t> occlusion_voxel_set;
//
////  bool getExplorationArea =
////      pipeline_->getExplorationCandidate(surface_voxel_set, occlusion_voxel_set);
////                                                               frontier_voxel_set,
////                                                               occlusion_voxel_set);
////  if (!getExplorationArea) { ROS_ERROR("no exploration area received "); }
//
//  visualization_msgs::Marker voxel_block_marker;
//  voxel_block_marker.header.frame_id = frame_id_;
//  voxel_block_marker.ns = frame_id_;
//  voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
//  voxel_block_marker.scale.x = res_;
//  voxel_block_marker.scale.y = res_;
//  voxel_block_marker.scale.z = res_;
//  voxel_block_marker.action = visualization_msgs::Marker::ADD;
//  voxel_block_marker.color.r = 0.0f;
//  voxel_block_marker.color.g = 0.0f;
//  voxel_block_marker.color.b = 1.0f;
//  voxel_block_marker.color.a = 1.0;
//
//  visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;
//  voxel_block_marker_msg.id = 0;
//  voxel_block_marker_msg.ns = "occlusion";
//  voxel_block_marker_msg.color.r = 1.0f;
//  voxel_block_marker_msg.color.g = 0.0f;
//  voxel_block_marker_msg.color.b = 0.0f;
//  voxel_block_marker_msg.color.a = 1.0;
//
//  visualization_msgs::Marker surface_voxels_msg = voxel_block_marker;
//  surface_voxels_msg.id = 0;
//  surface_voxels_msg.ns = "surface frontier";
//  surface_voxels_msg.lifetime = ros::Duration(6);
//  surface_voxels_msg.color.r = 1.0f;
//  surface_voxels_msg.color.g = 0.0f;
//  surface_voxels_msg.color.b = 1.0f;
//  surface_voxels_msg.color.a = 0.5;
//  visualization_msgs::Marker frontier_voxels_msg = voxel_block_marker;
//  frontier_voxels_msg.ns = "frontier";
//  frontier_voxels_msg.id = 0;
////  frontier_voxels_msg.lifetime = ros::Duration(10);
//  frontier_voxels_msg.color.r = 0.0f;
//  frontier_voxels_msg.color.g = 1.0f;
//  frontier_voxels_msg.color.b = 0.0f;
//  frontier_voxels_msg.color.a = 0.5f;
//
//  visualization_msgs::Marker occlusion_voxels_msg = voxel_block_marker;
//  occlusion_voxels_msg.ns = "occluded surface";
//  occlusion_voxels_msg.id = 0;
////  occlusion_voxels_msg.lifetime = ros::Duration(10);
//  occlusion_voxels_msg.color.r = 1.0f;
//  occlusion_voxels_msg.color.g = 1.0f;
//  occlusion_voxels_msg.color.b = 0.0f;
//  occlusion_voxels_msg.color.a = 0.5;
//
//  if (frame_ % N_frame_pub == 0) {
//    for (const auto &updated_block : updated_blocks) {
//      int morton_code = (int) compute_morton(updated_block[0], updated_block[1], updated_block[2]);
////      ROS_INFO("morton code updated %i ", morton_code );
////      std::cout << "updated block " << updated_block << std::endl;
//      vec3i occupied_block_voxels = node_it.getOccupiedVoxels(0.65, updated_block);
////      std::cout << "size occp blocks " << occupied_block_voxels.size() << std::endl;
//      voxel_block_map_[morton_code] = occupied_block_voxels;
//    }
//
//
//    /**
//     * FRONTIER
//     */
//    for (const auto &frontier_block : frontier_blocks) {
//      int morton_code =
//          (int) compute_morton(frontier_block[0], frontier_block[1], frontier_block[2]);
////      ROS_INFO("morton code updated %i ", morton_code );
////      std::cout << "frontier block \n" << frontier_block << std::endl;
//
//      vec3i frontier_block_voxels = node_it.getFrontierVoxels(0.1f, frontier_block);
////      std::cout << "size frontier blocks " << frontier_block_voxels.size() << std::endl;
//      frontier_voxel_map_[morton_code] = frontier_block_voxels;
//    }
//    for (const auto &frontier_block : frontier_blocks_map) {
//      vec3i frontier_block_voxels = node_it.getFrontierVoxels(0.1f, frontier_block.second);
//      surface_voxel_map_[frontier_block.first] = frontier_block_voxels;
//    }
//    for (const auto &occl_block : occlusion_blocks) {
//      int morton_code = (int) compute_morton(occl_block[0], occl_block[1], occl_block[2]);
//      vec3i occl_block_voxels = node_it.getOccludedVoxels(0.1f, occl_block);
//      occlusion_voxel_map_[morton_code] = occl_block_voxels;
//    }
//    /**
//     * SURFACE
//     */
///*    for (auto it = surface_voxel_set.begin(); it != surface_voxel_set.end(); ++it) {
////      std::cout << *it << std::endl;
//      Eigen::Vector3i surface_blocks = unpack_morton(*it);
////      std::cout << "surface block \n"<< surface_blocks << std::endl;
//      // TODO problem with getting occupied voxels
//      surface_voxel_map_[*it] = node_it.getOccupiedVoxels(0.5, surface_blocks);
////      if(frontier_voxel_set.find(*it)!=frontier_voxel_set.end()){
////        ROS_INFO("%i also in frontier ", *it);
////      }// morton code
//    }*/
//
////    for(auto it = frontier_voxel_set.begin(); it!= frontier_voxel_set.end(); ++it){
////      Eigen::Vector3i frontier_blocks = unpack_morton(*it);
////      frontier_voxel_map_[*it] = node_it.getOccupiedVoxels(0.1, frontier_blocks);
////    }
//
//  }
//
//  for (auto voxel_block = voxel_block_map_.begin(); voxel_block != voxel_block_map_.end();
//       voxel_block++) {
//    for (const auto &occupied_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) occupied_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) occupied_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) occupied_voxel[2] + 0.5) * res_;
//
//      voxel_block_marker_msg.points.push_back(cube_center);
//    }
//  }
//  for (auto voxel_block = surface_voxel_map_.begin(); voxel_block != surface_voxel_map_.end();
//       voxel_block++) {
//    for (const auto &surface_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) surface_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) surface_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) surface_voxel[2] + 0.5) * res_;
//
//      surface_voxels_msg.points.push_back(cube_center);
//    }
//  }
//
//  for (auto voxel_block = frontier_voxel_map_.begin(); voxel_block != frontier_voxel_map_.end();
//       voxel_block++) {
//    for (const auto &frontier_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) frontier_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) frontier_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) frontier_voxel[2] + 0.5) * res_;
//
//      frontier_voxels_msg.points.push_back(cube_center);
//    }
//  }
//
//  for (auto voxel_block = occlusion_voxel_map_.begin(); voxel_block != occlusion_voxel_map_.end();
//       voxel_block++) {
//    for (const auto &occl_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) occl_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) occl_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) occl_voxel[2] + 0.5) * res_;
//
//      occlusion_voxels_msg.points.push_back(cube_center);
//    }
//  }
//  boundary_marker_pub_.publish(surface_voxels_msg);
//  frontier_marker_pub_.publish(frontier_voxels_msg);
//  frontier_marker_pub_.publish(occlusion_voxels_msg);
//  block_based_marker_pub_.publish(voxel_block_marker_msg);
//};

//void SupereightNode::visualizeMapSDF(vec3i &occupied_voxels,
//                                        vec3i &freed_voxels,
//                                        vec3i &updated_blocks) {
////void SupereightNode::visualizeMapSDF(std::vector<Eigen::Vector3i>& occupied_voxels,
////                                        std::vector<Eigen::Vector3i>& freed_voxels,
////                                        std::vector<Eigen::Vector3i>& updated_blocks) {
//  // publish every N-th frame
//  int N_frame_pub = 1;
//
//  pipeline_->getMap(octree_);
//  node_iterator<T> node_it(*octree_);
//
//  if (pub_map_update_) {
//    visualization_msgs::Marker map_marker_msg;
//
//    vec3i surface_voxels = node_it.getSurfaceVoxels();
//
//    map_marker_msg.header.frame_id = frame_id_;
//    map_marker_msg.ns = frame_id_;
//    map_marker_msg.id = 0;
//    map_marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
//    map_marker_msg.scale.x = res_;
//    map_marker_msg.scale.y = res_;
//    map_marker_msg.scale.z = res_;
//    map_marker_msg.action = visualization_msgs::Marker::ADD;
//    map_marker_msg.color.r = 1.0f;
//    map_marker_msg.color.g = 1.0f;
//    map_marker_msg.color.b = 0.0f;
//    map_marker_msg.color.a = 1.0;
//
//    for (const auto &surface_voxel : surface_voxels) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) surface_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) surface_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) surface_voxel[2] + 0.5) * res_;
//
//      map_marker_msg.points.push_back(cube_center);
//    }
//
//    if (frame_ % N_frame_pub == 0) {
//      map_marker_pub_.publish(map_marker_msg);
//    }
//  }
//    constexpr bool is_map_block_based = true;
////  if (is_map_block_based) {
////    visualization_msgs::Marker voxel_block_marker;
////    voxel_block_marker.header.frame_id = frame_id_;
////    voxel_block_marker.ns = frame_id_;
////    voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
////    voxel_block_marker.scale.x = res_;
////    voxel_block_marker.scale.y = res_;
////    voxel_block_marker.scale.z = res_;
////    voxel_block_marker.action = visualization_msgs::Marker::ADD;
////    voxel_block_marker.color.r = 0.0f;
////    voxel_block_marker.color.g = 0.0f;
////    voxel_block_marker.color.b = 1.0f;
////    voxel_block_marker.color.a = 1.0;
////
////    visualization_msgs::MarkerArray voxel_block_marker_array_msg;
////    visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;
////
////    if (pub_block_based_marker_) {
////      voxel_block_marker_msg.id = 0;
////      voxel_block_marker_msg.color.r = 1.0f;
////      voxel_block_marker_msg.color.g = 0.0f;
////      voxel_block_marker_msg.color.b = 0.0f;
////      voxel_block_marker_msg.color.a = 1.0;
////    }
////
////    if (frame_ % N_frame_pub == 0) {
////      for (const auto &updated_block : updated_blocks) {
////        int morten_code = (int)compute_morton(
////            updated_block[0], updated_block[1], updated_block[2]);
////
////        std::vector<Eigen::Vector3i> occupied_block_voxels =
////            node_it.getSurfaceVoxels(0.25, updated_block);
////
////        if (pub_block_based_marker_array_) {
////          voxel_block_marker.id = morten_code;
////          voxel_block_marker.points.clear();
////
////          for (const auto &occupied_voxel : occupied_block_voxels) {
////            geometry_msgs::Point cube_center;
////            cube_center.x =
////                (static_cast<double>(occupied_voxel[0]) + 0.5) * res_;
////            cube_center.y =
////                (static_cast<double>(occupied_voxel[1]) + 0.5) * res_;
////            cube_center.z =
////                (static_cast<double>(occupied_voxel[2]) + 0.5) * res_;
////            voxel_block_marker.points.push_back(cube_center);
////          }
////          voxel_block_marker_array_msg.markers.push_back(voxel_block_marker);
////        }
////
////        if (pub_block_based_marker_) {
////          voxel_block_map_[morten_code] = occupied_block_voxels;
////        }
////      }
////
////      if (pub_block_based_marker_array_) {
////        block_based_marker_array_pub_.publish(voxel_block_marker_array_msg);
////      }
////    }
////
////    if (pub_block_based_marker_) {
////      for (auto voxel_block = voxel_block_map_.begin();
////           voxel_block != voxel_block_map_.end(); voxel_block++) {
////        for (const auto &occupied_voxel : voxel_block->second) {
////          geometry_msgs::Point cube_center;
////
////          cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
////          cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
////          cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;
////
////          voxel_block_marker_msg.points.push_back(cube_center);
////        }
////      }
////      block_based_marker_pub_.publish(voxel_block_marker_msg);
////    }
////  }
//};



/* Taken from https://github.com/ethz-asl/volumetric_mapping */
//std_msgs::ColorRGBA SupereightNode::percentToColor(double h) {
//  /* Helen's note: direct copy from OctomapProvider. */
//  std_msgs::ColorRGBA color;
//  color.a = 1.0;
//  /* blend over HSV-values (more colors) */
//
//  double s = 1.0;
//  double v = 1.0;
//
//  h -= floor(h);
//  h *= 6;
//  int i;
//  double m, n, f;
//
//  i = floor(h);
//  f = h - i;
//  if (!(i & 1)) f = 1 - f; /* if i is even */
//  m = v * (1 - s);
//  n = v * (1 - s * f);
//
//  switch (i) {
//    case 6:
//    case 0:color.r = v;
//      color.g = n;
//      color.b = m;
//      break;
//    case 1:color.r = n;
//      color.g = v;
//      color.b = m;
//      break;
//    case 2:color.r = m;
//      color.g = v;
//      color.b = n;
//      break;
//    case 3:color.r = m;
//      color.g = n;
//      color.b = v;
//      break;
//    case 4:color.r = n;
//      color.g = m;
//      color.b = v;
//      break;
//    case 5:color.r = v;
//      color.g = m;
//      color.b = n;
//      break;
//    default:color.r = 1;
//      color.g = 0.5;
//      color.b = 0.5;
//      break;
//  }
//
//  return color;
//}

}  // namespace se

