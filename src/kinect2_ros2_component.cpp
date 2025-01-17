#include "kinect2_ros2/kinect2_ros2_component.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace kinect2_ros2
{
static cv::Mat _depth_image(cv::Mat::zeros(cv::Size(512, 424), CV_16UC1));
static cv::Mat _rgb_image(cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC3));

static libfreenect2::Frame * _freenect_depth_pointer = nullptr;
static libfreenect2::Frame * _freenect_rgb_pointer = nullptr;

static rclcpp::Time _depth_stamp;
static rclcpp::Time _rgb_stamp;

static bool _depth_flag;
static bool _rgb_flag;

Kinect2RosComponent::Kinect2RosComponent(const rclcpp::NodeOptions & options)
: Node("kinect2_ros2", options),
  color_frame_listener_(get_logger()),
  depth_frame_listener_(get_logger()),
  depth_registration_(false)
{
  timer_ = create_wall_timer(1ms, std::bind(&Kinect2RosComponent::timer_callback, this));
  
  std::string pkg_share = ament_index_cpp::get_package_share_directory("kinect2_ros2");

  //todo: use parameters
  depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect2",
    "file://" + pkg_share + "/cfg/calibration_depth.yaml");

  rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect2",
    "file://" + pkg_share + "/cfg/calibration_rgb.yaml");

  depth_registration_ = this->declare_parameter("depth_registration", depth_registration_);

  rgb_info_ = rgb_info_manager_->getCameraInfo();
  rgb_info_.header.frame_id = "kinect2_rgb";
  depth_info_ = depth_info_manager_->getCameraInfo();
  if(depth_registration_) {
    depth_info_.header.frame_id = rgb_info_.header.frame_id;
  }
  else {
    depth_info_.header.frame_id = "kinect2_depth";
  }

  depth_pub_ = image_transport::create_camera_publisher(this, depth_registration_?"depth_registered/image_raw":"depth/image_raw");
  rgb_pub_ = image_transport::create_camera_publisher(this, "rgb/image_raw");

  int num_devices = fn_ctx_.enumerateDevices();
  if (num_devices == 0) {
    RCLCPP_ERROR(get_logger(), "FREENECT2 - NO DEVICES");
    rclcpp::shutdown();
  }
  fn_dev_ = fn_ctx_.openDevice(0); // default open first device
  if (fn_dev_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "FREENECT2 - ERROR OPEN");
    rclcpp::shutdown();
  }

  fn_dev_->setColorFrameListener(&color_frame_listener_);
  fn_dev_->setIrAndDepthFrameListener(&depth_frame_listener_);

  bool success = fn_dev_->start();
  if (success == false) {
    RCLCPP_ERROR(get_logger(), "FREENECT2 - ERROR START STREAMS");
    rclcpp::shutdown();
  }
}

Kinect2RosComponent::~Kinect2RosComponent()
{
  RCLCPP_INFO(get_logger(), "stopping kinect2");
  fn_dev_->stop();
  fn_dev_->close();
}


/* The freenect lib stores the depth data in a static region of the memory, so the best way to
create a cv::Mat is passing the pointer of that region, avoiding the need of copying the data
to a new cv::Mat. This way, the callback only used to set a flag that indicates that a new image
has arrived. The flag is unset when a msg is published */
bool Kinect2RosComponent::IrAndDepthFrameListener::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame)
{
  if (_depth_flag || type != libfreenect2::Frame::Type::Depth) { // ignore IR
    return false;
  }

  // RCLCPP_INFO(_logger, "Depth width %lu height %lu", frame->width, frame->height);
  // RCLCPP_INFO(_logger, "Depth format %d", frame->format);

  if (_freenect_depth_pointer != frame) {
    // Format: Float = 2, A 4-byte float per pixel, -> CV_32FC1
    cv::Mat depth_32FC1(frame->height, frame->width, CV_32FC1, frame->data);
    depth_32FC1.convertTo(_depth_image, CV_16UC1); //convert 32bit to 16bit
    cv::flip(_depth_image, _depth_image, 1);
    _freenect_depth_pointer = frame;
  }

  _depth_stamp =  rclcpp::Clock{RCL_SYSTEM_TIME}.now();
  _depth_flag = true;

  // take ownership of the frame, otherwise the pointer won't be valid anymore
  // don't forget to delete it
  return true;
}

bool Kinect2RosComponent::ColorFrameListener::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame)
{
  if (_rgb_flag || type != libfreenect2::Frame::Type::Color) {
    return false;
  }

  // RCLCPP_INFO(_logger, "Color width %lu height %lu", frame->width, frame->height);
  // RCLCPP_INFO(_logger, "Color format %d", frame->format);

  if (_freenect_rgb_pointer != frame) {
    // Format: BGRX = 4, 4 bytes of B, G, R, and unused per pixel -> CV_8UC4
    cv::Mat bgra_image(frame->height, frame->width, CV_8UC4, frame->data);
    cvtColor(bgra_image, _rgb_image, CV_BGRA2RGB); // convert to rgb
    _freenect_rgb_pointer = frame;
  }

   _rgb_stamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
  _rgb_flag = true;

  // take ownership of the frame, otherwise the pointer won't be valid anymore
  // don't forget to delete it
  return true;
}

void Kinect2RosComponent::timer_callback()
{

  if (_depth_flag && _freenect_depth_pointer) {
    depth_info_.header.stamp = _depth_stamp;
    auto msg = cv_bridge::CvImage(depth_info_.header, "16UC1", _depth_image).toImageMsg();
    depth_pub_.publish(*msg, depth_info_);
    if (_freenect_depth_pointer)
    {
      delete _freenect_depth_pointer;
      _freenect_depth_pointer = nullptr;
    }

    // cv::imshow("Depth", _depth_image);
    // cv::waitKey(1);
    _depth_flag = false;
  }

  if (_rgb_flag && _freenect_rgb_pointer) {
    rgb_info_.header.stamp = _rgb_stamp;
    auto msg = cv_bridge::CvImage(rgb_info_.header, "rgb8", _rgb_image).toImageMsg();
    rgb_pub_.publish(*msg, rgb_info_);
    if (_freenect_rgb_pointer)
    {
      delete _freenect_rgb_pointer;
      _freenect_rgb_pointer = nullptr;
    }

    // cv::imshow("RGB", _rgb_image);
    // cv::waitKey(1);
    _rgb_flag = false;
  }
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kinect2_ros2::Kinect2RosComponent)
