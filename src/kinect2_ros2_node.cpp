#include "rclcpp/rclcpp.hpp"
#include "kinect2_ros2/kinect2_ros2_component.hpp"
#include "class_loader/class_loader.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "rclcpp_components/node_factory.hpp"

std::vector<class_loader::ClassLoader *> _loaders;
std::vector<rclcpp_components::NodeInstanceWrapper> _node_wrappers;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_depth_image_proc_component(bool depth_registered)
{
  auto path_prefix = ament_index_cpp::get_package_prefix("depth_image_proc");
  auto loader = new class_loader::ClassLoader(path_prefix + "/lib/libdepth_image_proc.so");
  auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  std::vector<std::string> arguments {
    "image_rect:=depth/image_raw",
    "camera_info:=depth/camera_info",
    "rgb/image_rect_color:=rgb/image_raw",
    "depth_registered/image_rect:=depth_registered/image_raw"
  };
  options.arguments(arguments);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node;
  for (auto clazz : classes) {
    auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
    auto wrapper = node_factory->create_node_instance(options);
    if(depth_registered) {
      if (clazz == "rclcpp_components::NodeFactoryTemplate<depth_image_proc::PointCloudXyzrgbNode>") {
        node = wrapper.get_node_base_interface();
        _node_wrappers.push_back(wrapper);
        break;
      }
    }
    else {
      if (clazz == "rclcpp_components::NodeFactoryTemplate<depth_image_proc::PointCloudXyzNode>") {
        node = wrapper.get_node_base_interface();
        _node_wrappers.push_back(wrapper);
        break;
      }
    }
  }
  if (node != nullptr) {
    _loaders.push_back(loader);
    return node;
  }
  throw std::invalid_argument("depth_image_proc not found");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto kinect2_component = std::make_shared<kinect2_ros2::Kinect2RosComponent>(options);
  auto depth_image_proc_component = get_depth_image_proc_component(kinect2_component->depthRegistered());

  exec.add_node(kinect2_component);
  exec.add_node(depth_image_proc_component);

  exec.spin();

  for (auto wrapper : _node_wrappers) {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  _node_wrappers.clear();

  rclcpp::shutdown();

  return 0;
}
