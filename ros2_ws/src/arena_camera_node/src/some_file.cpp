#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ArenaApi.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    RCLCPP_DEBUG(this->get_logger(),
                 std::string("Creating \"") + this->get_name() + "\" node");

    this->declare_parameter("serial", "");
    this->declare_parameter("topic",
                            std::string("/") + this->get_name() + "/images");
    this->declare_parameter("gain", -1);
    this->declare_parameter("width", -1);
    this->declare_parameter("height", -1);
    this->declare_parameter("pixelformat", "");
    this->declare_parameter("exposure_auto", true);
    this->declare_parameter("exposure_time", -1);
    this->declare_parameter("trigger_mode", false);
  }
  void run()
  {
    // device -------------------------------------------
    this->wait_until_a_device_is_discovered_();
    /*
    this->_create_device();

    // nodes --------------------------------------------
    this->_set_nodes();

    // services -----------------------------------------

    this->_srv = this->create_service(Trigger, 'trigger_image',
                                      this->_publish_images_at_trigger);
    // streaming ----------------------------------------
    this->_device->start_stream();

    // publish images -------------------------------
    if (!(this->trigger_mode_active)) {
      this->_publish_images();
    }
    */
  };

 private:
  void wait_until_a_device_is_discovered_()
  {
    pSystem_ = Arena::OpenSystem();
    auto device_infos = std::vector<Arena::DeviceInfo>();
    /*
      while (!device_infos.size()) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
                       "Interrupted while waiting for arena camera. Exiting.");
          rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(),
                    "No arena camera is discovered. Waiting for a camera to be "
                    "connected!");
        pSystem_->UpdateDevices(100);  // in millisec
        device_infos = pSystem_->GetDevices();
      }

      RCLCPP_INFO(this->get_logger(), "Discovered %d devices",
                  std::to_string(device_infos.size()));
      */
  };

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  Arena::ISystem* pSystem_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArenaCameraNode>());
  rclcpp::shutdown();
  return 0;
}