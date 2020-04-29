
#include "ArenaCameraNode.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<ArenaCameraNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}