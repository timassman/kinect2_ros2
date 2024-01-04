#ifndef KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_
#define KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_

#include "libfreenect2/libfreenect2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"

namespace kinect2_ros2
{

class Kinect2RosComponent : public rclcpp::Node
{
public:
  Kinect2RosComponent(const rclcpp::NodeOptions & options);
  ~Kinect2RosComponent();

  bool depthRegistered() const {return depth_registration_;}

private:
  class ColorFrameListener : public libfreenect2::FrameListener
  {
    public:
      ColorFrameListener(rclcpp::Logger logger)
      : _logger(logger)
      {}
    private:
      rclcpp::Logger _logger;
  /**
   * libfreenect2 calls this function when a new frame is decoded.
   * @param type Type of the new frame.
   * @param frame Data of the frame.
   * @return true if you want to take ownership of the frame, i.e. reuse/delete it. Will be reused/deleted by caller otherwise.
   */
    bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;
  };

  class IrAndDepthFrameListener : public libfreenect2::FrameListener
  {
    public:
      IrAndDepthFrameListener(rclcpp::Logger logger)
      : _logger(logger)
      {}
    private:
      rclcpp::Logger _logger;
  /**
   * libfreenect2 calls this function when a new frame is decoded.
   * @param type Type of the new frame.
   * @param frame Data of the frame.
   * @return true if you want to take ownership of the frame, i.e. reuse/delete it. Will be reused/deleted by caller otherwise.
   */
    bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;
  };

  ColorFrameListener color_frame_listener_;
  IrAndDepthFrameListener depth_frame_listener_;

  libfreenect2::Freenect2 fn_ctx_;
  libfreenect2::Freenect2Device * fn_dev_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, depth_info_manager_;
  sensor_msgs::msg::CameraInfo rgb_info_, depth_info_;

  image_transport::CameraPublisher depth_pub_, rgb_pub_;

  void timer_callback();

  bool depth_registration_;

};

}
#endif  // KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_
