// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>  // WallTimer
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>  // Trigger

// arena sdk
#include "ArenaApi.h"

using namespace std::chrono_literals;

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
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
            m_pSystem->DestroyDevice(pDevice);
            RCLCPP_INFO(this->get_logger(), "Device is destroyed");
          }
        });

    //
    // PARAMS -----------------------------------------------------------------
    //
    this->declare_parameter("serial", "");
    this->declare_parameter("topic",
                            std::string("/") + this->get_name() + "/images");
    this->declare_parameter("gain", -1.0);
    this->declare_parameter("width", -1);
    this->declare_parameter("height", -1);
    this->declare_parameter("pixelformat", "");
    this->declare_parameter("exposure_auto", true);
    this->declare_parameter("exposure_time", -1.0);
    this->declare_parameter("trigger_mode", false);

    //
    // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
    //
    // TODO
    // - Think of design that allow the node to start stream as soon as
    // it is initialized without waiting for spin to be called
    m_wait_for_device_timer_callback_ = this->create_wall_timer(
        1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

    //
    // TRIGGER (service) ------------------------------------------------------
    //
    using namespace std::placeholders;
    m_srv_ = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()) + "/trigger_image",
        std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1,
                  _2));

    //
    // Publisher --------------------------------------------------------------
    //
    rclcpp::SensorDataQoS qos;
    m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string(), qos);

    RCLCPP_INFO(this->get_logger(),
                std::string("Created \"") + this->get_name() + "\" node");
  }
  ~ArenaCameraNode() {}

  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

 private:
  void wait_for_device_timer_callback_();
  void run_();
  // TODO :
  // - cmdline serial for device to create
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void publish_images_();
  void publish_an_image_on_trigger_(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  sensor_msgs::msg::Image msg_form_image_(Arena::IImage* pImage);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_srv_;
};
