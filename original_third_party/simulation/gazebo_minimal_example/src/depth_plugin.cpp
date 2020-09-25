#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <ignition/math/Vector3.hh>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace gazebo {

struct MyDepthPlugin : public SensorPlugin {
  // Gazebo
  sdf::ElementPtr sdf_;
  event::ConnectionPtr depth_conn_;
  event::ConnectionPtr rgb_conn_;
  rendering::ScenePtr scene_;
  sensors::DepthCameraSensorPtr sensor_;
  rendering::DepthCameraPtr depth_camera_;

  // ROS
  size_t seq_ = 0;
  std::string depth_topic_ = "depth";
  std::string rgb_topic_ = "rgb";
  std::thread ros_thread_;
  ros::NodeHandle *ros_nh_;
  image_transport::ImageTransport *img_transport_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher rgb_pub_;

  MyDepthPlugin() {}
  ~MyDepthPlugin() {
    delete ros_nh_;
    delete img_transport_;
  }

  void Load(sensors::SensorPtr sptr, sdf::ElementPtr sdf) {
    sdf_ = sdf;

    // Load sensor pointer
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sptr);
    if (!sensor_) {
      gzerr << "MyDepthPlugin requires a DepthSensor.\n";
    }

    // Load depth camera
    depth_camera_ = sensor_->DepthCamera();
    if (!depth_camera_) {
      gzerr << "MyDepthPlugin not attached to a camera sensor!\n";
      return;
    }

    // Register depth callback
    auto depth_cb = std::bind(&MyDepthPlugin::depth_update, this);
    depth_conn_ = depth_camera_->ConnectNewDepthFrame(depth_cb);

    // Register rgb callback
    auto rgb_cb = std::bind(&MyDepthPlugin::rgb_update,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3,
                            std::placeholders::_4,
                            std::placeholders::_5);
    rgb_conn_ = depth_camera_->ConnectNewImageFrame(rgb_cb);

    // Create ROS thread
    ros_thread_ = std::thread(&MyDepthPlugin::ros_thread, this);
  }

  void ros_thread() {
    // Initialize ros node
    if (ros::isInitialized() == false) {
      ROS_FATAL("ROS is not initialized!");
    }
    ros_nh_ = new ros::NodeHandle();

    // Get rostopic names from sdf file
    if (sdf_->HasElement("depth_topic")) {
      depth_topic_ = sdf_->Get<std::string>("depth_topic");
    }
    if (sdf_->HasElement("rgb_topic")) {
      rgb_topic_ = sdf_->Get<std::string>("rgb_topic");
    }

    // Register publisher
    int queue_size = 1;
    img_transport_ = new image_transport::ImageTransport(*ros_nh_);
    depth_pub_ = img_transport_->advertise(depth_topic_, 1);
    rgb_pub_ = img_transport_->advertise(rgb_topic_, 1);
  }

  void depth_update() {
    // Get sim time
    rendering::ScenePtr scene = depth_camera_->GetScene();
    common::Time timestamp = scene->SimTime();

    // Get data dimensions
    const int image_width = depth_camera_->ImageWidth();
    const int image_height = depth_camera_->ImageHeight();

    // Convert data to cv::Mat
    cv::Mat depth_image(image_height, image_width, CV_32FC1,
                        (void *) depth_camera_->DepthData());

    // Form std_msgs::Header
    std_msgs::Header header;
    header.seq = seq_;
    header.stamp = ros::Time(timestamp.sec, timestamp.nsec);
    header.frame_id = depth_topic_;

    // Form sensor_msgs::Image for depth
    cv_bridge::CvImage depth_ros;
    depth_ros.header = header;
    depth_ros.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_ros.image = depth_image;

    // Publish depth
    depth_pub_.publish(depth_ros.toImageMsg());
  }

  void rgb_update(const unsigned char *image_raw,
                  const int image_width,
                  const int image_height,
                  const int image_depth,
                  const std::string &format) {
    // Get sim time
    rendering::ScenePtr scene = depth_camera_->GetScene();
    common::Time timestamp = scene->SimTime();

    // Convert data to cv::Mat
    const int buffer_size = image_width * image_height * 3;
    unsigned char *buffer = new unsigned char[buffer_size + 1];
    memcpy((char *) buffer, image_raw, buffer_size);
    const cv::Mat image(image_height, image_width, CV_8UC3, buffer);

    // Build std_msgs::Header
    std_msgs::Header header;
    header.seq = seq_;
    header.stamp = ros::Time(timestamp.sec, timestamp.nsec);
    header.frame_id = rgb_topic_;

    // Build sensor_msgs::Image
    const auto img_msg = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();

    // Publish image and clean up
    rgb_pub_.publish(img_msg);
    delete[] buffer;
  }
};

GZ_REGISTER_SENSOR_PLUGIN(MyDepthPlugin)
} // namespace gazebo
