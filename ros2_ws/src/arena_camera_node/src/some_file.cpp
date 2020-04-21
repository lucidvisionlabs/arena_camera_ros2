#include <chrono>
//#include <functional>
//#include <memory>
#include <string>

#include "ArenaApi.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"  // WallTimer
//#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
/*
namespace Arena
{
class SafeISystemPtr : public Arena::Isystem
{
  // shared pointer for Arena::ISystem
 public:
  SafeISystemPtr(pSystem)
  {
    if (!pSystem){}

    pSystem_ =
        std::shared_ptr<Arena::ISystem>(nullptr, [](Arena::ISystem* pSystem) {
          if (pSystem) {  // this is an issue for multi devices
            Arena::CloseSystem(pSystem);
          }
        });
  }
  ~SafeISystemPtr() {}
  std::shared_ptr<Arena::ISystem> pSystem_;
};
}  // namespace Arena
*/

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    std::string f = "";

    RCLCPP_INFO(this->get_logger(),
                std::string("Creating \"") + this->get_name() + "\" node");

    // ARENASDK ---------------------------------------------------------------
    // Custom deleter for system
    pSystem_ =
        std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
          if (pSystem) {  // this is an issue for multi devices
            Arena::CloseSystem(pSystem);
            RCLCPP_INFO(this->get_logger(), "System is destroyed");
          }
        });
    pSystem_.reset(Arena::OpenSystem());

    // Custom deleter for device
    pDevice_ =
        std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
          if (pSystem_ && pDevice) {
            pSystem_->DestroyDevice(pDevice);
            RCLCPP_INFO(this->get_logger(), "Device is destroyed");
          }
        });

    //
    // PARAMS -----------------------------------------------------------------
    //
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

    //
    // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
    //
    // update devices is a blocking call and we need to get to spin;
    // use Node::create_wall_timer() to get non blocking call
    pSystem_->UpdateDevices(100);  // in millisec
    auto device_infos = pSystem_->GetDevices();

    // no device is connected
    // TODO :
    // - have in seperate palce ?
    if (device_infos.size()) {
      // fn to check if the device has arrived
      auto discover_device_callback = [this]() {
        // something happend while checking for cameras
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
                       "Interrupted while waiting for arena camera. Exiting.");
          rclcpp::shutdown();
        }

        // camera discovery
        pSystem_->UpdateDevices(100);  // in millisec
        auto device_infos_ = pSystem_->GetDevices();

        // no camera is connected
        if (device_infos_.size()) {
          RCLCPP_INFO(this->get_logger(),
                      "No arena camera is connected. Waiting for device ...");
        }
        // at least on is found
        else {
          // disable timer and its callback
          while (!discover_devices_timer_->is_canceled()) {
            discover_devices_timer_->cancel();
          }
          RCLCPP_INFO(this->get_logger(),
                      "%d arena device(s) has been discoved.",
                      device_infos_.size());
          run()
        }
      };
      discover_devices_timer_ =
          this->create_wall_timer(2s, discover_device_callback);
    }
    //
    // TRIGGER (service)
    //
    auto serv = this->create_service<rclcpp::>("trigger_image");
  };
  ~ArenaCameraNode(){};
  void run()
  {
    RCLCPP_INFO(this->get_logger(), " run()");

    // device -------------------------------------------

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
    RCLCPP_INFO(this->get_logger(), " run() done ");
  };

 private:
  void wait_until_a_device_is_discovered_() {}
  // rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // size_t count_;
  std::shared_ptr<Arena::ISystem> pSystem_;
  std::shared_ptr<Arena::IDevice> pDevice_;
  rclcpp::TimerBase::SharedPtr discover_devices_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<ArenaCameraNode>();
    node->run();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}