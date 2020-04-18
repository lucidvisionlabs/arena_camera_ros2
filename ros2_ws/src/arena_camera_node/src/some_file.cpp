#include <chrono>
//#include <functional>
//#include <memory>
#include <string>

#include "ArenaApi.h"
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode()
      : Node("arena_camera_node"), pSystem_(nullptr), pDevice_(nullptr)
  {
    RCLCPP_INFO(this->get_logger(),
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

    run();
  };
  ~ArenaCameraNode()
  {
    // pSystem_->DestroyDevice(pDevice_);
    Arena::CloseSystem(pSystem_);
  };
  void run()
  {
    RCLCPP_INFO(this->get_logger(), " run()");

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
    RCLCPP_INFO(this->get_logger(), "-> ", std::string(__FILE__), "<=");
    RCLCPP_INFO(this->get_logger(), " run() done ");
  };

 private:
  void wait_until_a_device_is_discovered_()
  {
    // update devices is a blocking call and we need to get to spin;
    // use Node::create_wall_timer() to get non blocking call
    pSystem_ = Arena::OpenSystem();
    auto device_infos = std::vector<Arena::DeviceInfo>();
    pSystem_->UpdateDevices(100);  // in millisec
    device_infos = pSystem_->GetDevices();

    auto check = []() { std::cout << "Lambda" << '\n'; };

    auto timer = this->create_wall_timer(2s, check);

    /*
      while (device_infos.size()) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
                       "Interrupted while waiting for arena camera. Exiting.");
          rclcpp::shutdown();
          return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "No arena camera is discovered. Waiting for a camera to be "
                    "connected!");
        pSystem_->UpdateDevices(wait_time_millsec);
        device_infos = pSystem_->GetDevices();
      }

      RCLCPP_INFO(this->get_logger(), "Discovered %d devices",
                  device_infos.size());
    */
  }
  // rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // size_t count_;
  Arena::ISystem* pSystem_;
  Arena::IDevice* pDevice_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArenaCameraNode>());
  rclcpp::shutdown();
  return 0;
}