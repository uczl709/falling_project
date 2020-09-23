#include <functional>

#include <Eigen/Dense>
//#include <Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gui/model/ModelEditorEvents.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace gazebo {

struct MyPosePlugin : ModelPlugin {
  physics::ModelPtr model_;
  physics::ModelPtr duck_model_;
  sdf::ElementPtr sdf_;
  event::ConnectionPtr conn_;

  // ROS
  bool ros_thread_running_ = false;
  std::thread ros_th_;
  ros::NodeHandle *ros_nh_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_obj_pub_;
  ros::Subscriber pose_sub_;
  size_t ros_seq_ = 0;
  std::string frame_id_ = "";

  MyPosePlugin() {}
  ~MyPosePlugin() { delete ros_nh_; }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    // Gazebo things
    model_ = parent;

    sdf_ = sdf;
    conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MyPosePlugin::on_update, this));

    // Start ROS thread
    ros_th_ = std::thread(&MyPosePlugin::ros_thread, this);
  }

  void ros_thread() {
    // Initialize ros node
    if (ros::isInitialized() == false) {
      ROS_FATAL("ROS is not initialized!");
    }
    ros_nh_ = new ros::NodeHandle();
    ROS_INFO("Initialize the pose_plugin cpp.");
    // Get frame id
    if (sdf_->HasElement("frame_id")) {
      frame_id_ = sdf_->Get<std::string>("frame_id");
    }

    // Register pose publisher
    {
      std::string topic_name = "pose";
      if (sdf_->HasElement("pose_topic")) {
        topic_name = sdf_->Get<std::string>("pose_topic");
      }
      pose_pub_ = ros_nh_->advertise<geometry_msgs::PoseStamped>(topic_name, 1);
    }

    {
      std::string topic_name = "poseinobj";
      if (sdf_->HasElement("pose_test_topic")) {
        topic_name = sdf_->Get<std::string>("pose_test_topic");
      }
      pose_obj_pub_ = ros_nh_->advertise<geometry_msgs::PoseStamped>(topic_name, 1);
    }

    // Register pose subscriber
    {
      std::string topic_name = "pose/set";
      int queue_size = 1;
      if (sdf_->HasElement("pose_set_topic")) {
        topic_name = sdf_->Get<std::string>("pose_set_topic");
      }
      pose_sub_ = ros_nh_->subscribe(topic_name,
                                     queue_size,
                                     &MyPosePlugin::pose_callback,
                                     this);
    }

    sleep(1);
    ros_thread_running_ = true;
  }

  void publish_pose() {

    // physics::WorldPtr world_ptr = model_->GetWorld();
    // physics::ModelPtr duck_model_ = world_ptr->ModelByName("duck");

    // const auto duckpose = duck_model_->WorldPose();
    // const auto campose = model_->WorldPose();
    
    // // form the duck_T_world:
    // Eigen::Matrix3f duckMat3 = Eigen::Quaternionf(duckpose.Rot().W(), duckpose.Rot().X(),
    //                                               duckpose.Rot().Y(), duckpose.Rot().Z()).toRotationMatrix();
    // Eigen::Matrix4f duckMat4 = Eigen::Matrix4f::Identity();
    // duckMat4.block(0,0,3,3) = duckMat3;
    // duckMat4(0,3) = duckpose.Pos().X(); 
    // duckMat4(1,3) = duckpose.Pos().Y(); 
    // duckMat4(2,3) = duckpose.Pos().Z();
    // // form the camera_T_world:
    // Eigen::Matrix3f camMat3 = Eigen::Quaternionf (campose.Rot().W(), campose.Rot().X(),
    //                                               campose.Rot().Y(), campose.Rot().Z()).toRotationMatrix();
    // Eigen::Matrix4f camMat4 = Eigen::Matrix4f::Identity();
    // camMat4.block(0,0,3,3) = camMat3;
    // camMat4(0,3) = campose.Pos().X(); 
    // camMat4(1,3) = campose.Pos().Y(); 
    // camMat4(2,3) = campose.Pos().Z(); 
    // //Mutiply:
    // Eigen::Matrix4f camInobjMat4 = duckMat4.inverse() * camMat4;
    // //ROS_INFO("The matrix:");
    // //Convert:
    // Eigen::Matrix3f camInobjMat3 = camInobjMat4.block(0,0,3,3);
    // Eigen::Quaternionf camInobjQua = Eigen::Quaternionf(camInobjMat3);
    // // put in msg:
    // geometry_msgs::PoseStamped msg;
    // msg.header.seq = ros_seq_;
    // msg.header.stamp = ros::Time(model_->GetWorld()->SimTime().Double());
    // msg.header.frame_id = frame_id_;
    // msg.pose.position.x = camInobjMat4(0,3);
    // msg.pose.position.y = camInobjMat4(1,3);
    // msg.pose.position.z = camInobjMat4(2,3);
    // msg.pose.orientation.w = camInobjQua.w();
    // msg.pose.orientation.x = camInobjQua.x();
    // msg.pose.orientation.y = camInobjQua.y();
    // msg.pose.orientation.z = camInobjQua.z();

    // // geometry_msgs::PoseStamped msg;
    // // msg.header.seq = ros_seq_;
    // // msg.header.stamp = ros::Time(model_->GetWorld()->SimTime().Double());
    // // msg.header.frame_id = frame_id_;
    // // msg.pose.position.x = campose.Pos().X();
    // // msg.pose.position.y = campose.Pos().Y();
    // // msg.pose.position.z = campose.Pos().Z();
    // // msg.pose.orientation.w = campose.Rot().W();
    // // msg.pose.orientation.x = campose.Rot().X();
    // // msg.pose.orientation.y = campose.Rot().Y();
    // // msg.pose.orientation.z = campose.Rot().Z();

    // pose_pub_.publish(msg);


    const auto pose = model_->WorldPose();
    geometry_msgs::PoseStamped msg;
    msg.header.seq = ros_seq_;
    msg.header.stamp = ros::Time(model_->GetWorld()->SimTime().Double());
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = pose.Pos().X();
    msg.pose.position.y = pose.Pos().Y();
    msg.pose.position.z = pose.Pos().Z();
    msg.pose.orientation.w = pose.Rot().W();
    msg.pose.orientation.x = pose.Rot().X();
    msg.pose.orientation.y = pose.Rot().Y();
    msg.pose.orientation.z = pose.Rot().Z();

    pose_pub_.publish(msg);
  }

  void publish_pose_obj(){
    
    physics::WorldPtr world_ptr = model_->GetWorld();
    physics::ModelPtr duck_model_ = world_ptr->ModelByName("duck");

    const auto duckpose = duck_model_->WorldPose();
    const auto campose = model_->WorldPose();
    
    // form the duck_T_world:
    Eigen::Matrix3f duckMat3 = Eigen::Quaternionf(duckpose.Rot().W(), duckpose.Rot().X(),
                                                  duckpose.Rot().Y(), duckpose.Rot().Z()).toRotationMatrix();
    Eigen::Matrix4f duckMat4 = Eigen::Matrix4f::Identity();
    duckMat4.block(0,0,3,3) = duckMat3;
    duckMat4(0,3) = duckpose.Pos().X(); 
    duckMat4(1,3) = duckpose.Pos().Y(); 
    duckMat4(2,3) = duckpose.Pos().Z();
    // form the camera_T_world:
    Eigen::Matrix3f camMat3 = Eigen::Quaternionf (campose.Rot().W(), campose.Rot().X(),
                                                  campose.Rot().Y(), campose.Rot().Z()).toRotationMatrix();
    Eigen::Matrix4f camMat4 = Eigen::Matrix4f::Identity();
    camMat4.block(0,0,3,3) = camMat3;
    camMat4(0,3) = campose.Pos().X(); 
    camMat4(1,3) = campose.Pos().Y(); 
    camMat4(2,3) = campose.Pos().Z(); 
    //Mutiply:
    Eigen::Matrix4f camInobjMat4 = duckMat4.inverse() * camMat4;
    //ROS_INFO("The matrix:");
    //Convert:
    Eigen::Matrix3f camInobjMat3 = camInobjMat4.block(0,0,3,3);
    Eigen::Quaternionf camInobjQua = Eigen::Quaternionf(camInobjMat3);
    // put in msg:
    geometry_msgs::PoseStamped msg;
    msg.header.seq = ros_seq_;
    msg.header.stamp = ros::Time(model_->GetWorld()->SimTime().Double());
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = camInobjMat4(0,3);
    msg.pose.position.y = camInobjMat4(1,3);
    msg.pose.position.z = camInobjMat4(2,3);
    msg.pose.orientation.w = camInobjQua.w();
    msg.pose.orientation.x = camInobjQua.x();
    msg.pose.orientation.y = camInobjQua.y();
    msg.pose.orientation.z = camInobjQua.z();

    // geometry_msgs::PoseStamped msg;
    // msg.header.seq = ros_seq_;
    // msg.header.stamp = ros::Time(model_->GetWorld()->SimTime().Double());
    // msg.header.frame_id = frame_id_;
    // msg.pose.position.x = campose.Pos().X();
    // msg.pose.position.y = campose.Pos().Y();
    // msg.pose.position.z = campose.Pos().Z();
    // msg.pose.orientation.w = campose.Rot().W();
    // msg.pose.orientation.x = campose.Rot().X();
    // msg.pose.orientation.y = campose.Rot().Y();
    // msg.pose.orientation.z = campose.Rot().Z();

    pose_obj_pub_.publish(msg);



    
    // geometry_msgs::PoseStamped msg;
    // msg.header.seq = ros_seq_;
    // msg.header.stamp = ros::Time(model_->GetWorld()->SimTime().Double());
    // msg.header.frame_id = frame_id_;
    // msg.pose.position.x = 0;
    // msg.pose.position.y = 0;
    // msg.pose.position.z = 0;
    // msg.pose.orientation.w = 1;
    // msg.pose.orientation.x = 0;
    // msg.pose.orientation.y = 0;
    // msg.pose.orientation.z = 0;

    // pose_obj_pub_.publish(msg);

  }

  void pose_callback(const geometry_msgs::Pose::ConstPtr &msg) {
    const double x = msg->position.x;
    const double y = msg->position.y;
    const double z = msg->position.z;
    const double qw = msg->orientation.w;
    const double qx = msg->orientation.x;
    const double qy = msg->orientation.y;
    const double qz = msg->orientation.z;
    ignition::math::Pose3d pose(x, y, z, qw, qx, qy, qz);
    model_->SetWorldPose(pose);
  }

  void on_update() {
    const auto pose = model_->WorldPose();
    model_->SetWorldPose(pose);

    if (ros_thread_running_) {
      publish_pose();
      publish_pose_obj();
      ros_seq_++;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(MyPosePlugin)
} // namespace gazebo
