#include "topic_statistics.hpp"

// int main(int argc, char * argv[]) {
//   rclcpp::init(argc, argv);
// //   rclcpp::spin(std::make_shared<orbbec_camera::tools::TopicStatistics>());
//   rclcpp::spin(std::make_shared<orbbec_camera::TopicStatistics>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::TopicStatistics)
