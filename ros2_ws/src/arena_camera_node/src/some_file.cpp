#include <chrono>      //chrono_literals
#include <functional>  // std::bind
#include <stdexcept>   // std::runtime_err
#include <string>

#include <ArenaApi.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>          // WallTimer
#include <std_srvs/srv/trigger.hpp>  // Trigger

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

    m_pSystem =
        std::shared_ptr<Arena::ISystem>(nullptr, [](Arena::ISystem* pSystem) {
          if (pSystem) {  // this is an issue for multi devices
            Arena::CloseSystem(pSystem);
          }
        });
  }
  ~SafeISystemPtr() {}
  std::shared_ptr<Arena::ISystem> m_pSystem;
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
    m_pSystem =
        std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
          if (pSystem) {  // this is an issue for multi devices
            Arena::CloseSystem(pSystem);
            RCLCPP_INFO(this->get_logger(), "System is destroyed");
          }
        });
    m_pSystem.reset(Arena::OpenSystem());

    // Custom deleter for device
    m_pDevice =
        std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
          if (m_pSystem && pDevice) {
            pSystem->DestroyDevice(pDevice);
            RCLCPP_INFO(this->get_logger(), "Device is destroyed");
          }
        });

    //
    // PARAMS -----------------------------------------------------------------
    //
    this->declare_parameter("serial", "");
    this->declare_parameter("topic",
                            std::string() + "/" + this->get_name() + "/images");
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
    m_pSystem->UpdateDevices(100);  // in millisec
    auto device_infos = m_pSystem->GetDevices();

    // no device is connected
    // TODO :
    // - have in separate place ?
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
        m_pSystem->UpdateDevices(100);  // in millisec
        auto device_infos_ = m_pSystem->GetDevices();

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
          run();
        }
      };
      discover_devices_timer_ =
          this->create_wall_timer(2s, discover_device_callback);
    }

    //
    // TRIGGER (service)
    //
    auto serv = this->create_service<std_srvs::srv::Trigger>(
        "trigger_image",
        // TODO : like to fn-trigger-image
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            -> void {
          RCLCPP_INFO(this->get_logger(),
                      "The trigger request is being served");
        });
  }
  ~ArenaCameraNode() {}
  void run()
  {
    RCLCPP_INFO(this->get_logger(), " run()");

    // device -------------------------------------------
    auto first_device = create_device_ros();
    m_pDevice.reset(first_device);

    // nodes --------------------------------------------
    set_nodes();

    // streaming ----------------------------------------
    m_pDevice->StartStream();

    // publish images -------------------------------
    // if (!(this->trigger_mode_active)) {
    publish_images();
    //}
    RCLCPP_INFO(this->get_logger(), " run() done ");
  }

  void publish_images() {}

  // TODO cmdline serialfor device to create
  Arena::IDevice* create_device_ros()
  {
    m_pSystem->UpdateDevices(100);  // in millisec
    auto device_infos = m_pSystem->GetDevices();
    if (device_infos.size()) {
      return m_pSystem->CreateDevice(device_infos.at(0));
    } else {
      // TODO: handel disconnection
      throw std::runtime_error(
          "camera(s) were disconnected after they were discovered");
    }
  }
  void set_nodes()
  {
    auto nodemap = m_pDevice->GetNodeMap();
    // device run on default profile all the time if no args are passed
    // otherwise, overwise only these params
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector",
                                           "Default");
    // execute the profile
    Arena::ExecuteNode(nodemap, "UserSetLoad");

    auto gain_value = this->get_parameter("gain").get_value();
    if (gain_value != -1) {
      Arena::SetNodeValue<double>(nodemap, "Gain", gain_value);
    }

    auto width_value = this->get_parameter("width").get_value();
    if (width_value != -1) {
      Arena::SetNodeValue<int64_t>(
          nodemap, "Width", width_value);  // TODO is this should be uint64_t
    }

    auto height_value = this->get_parameter("height").get_value();
    if (height_value != -1) {
      Arena::SetNodeValue<int64_t>(
          nodemap, "Height", height_value);  // TODO is this should be uint64_t
    }

    // TODO PIXEL FORMAT HANDLEING

    // TODO HANDEL EXPOSURE AUTO and TIME

    // HANDLE TRIGGER MODE
  }

  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

 private:
  void wait_until_a_device_is_discovered_() {}
  // rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // size_t count_;

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