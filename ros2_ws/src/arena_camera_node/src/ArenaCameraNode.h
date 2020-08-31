#pragma once

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
    log_info(std::string("Creating \"") + this->get_name() + "\" node");

    // ARENASDK ---------------------------------------------------------------
    // Custom deleter for system
    m_pSystem =
        std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
          if (pSystem) {  // this is an issue for multi devices
            Arena::CloseSystem(pSystem);
            log_info("System is destroyed");
          }
        });
    m_pSystem.reset(Arena::OpenSystem());

    // Custom deleter for device
    m_pDevice =
        std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
          if (m_pSystem && pDevice) {
            m_pSystem->DestroyDevice(pDevice);
            log_info("Device is destroyed");
          }
        });

    //
    // PARAMS -----------------------------------------------------------------
    //
    // TODO move t ofunction parse params like in the ros2 demos
    // https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
    std::string nextParameterToDeclare = "";
    try {
      nextParameterToDeclare = "serial";
      serial_ = this->declare_parameter<std::string>("serial", "");
      is_passed_serial_ = serial_ != "";

      nextParameterToDeclare = "pixelformat";
      pixelformat_ros_ = this->declare_parameter("pixelformat", "");
      is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

      nextParameterToDeclare = "width";
      width_ = this->declare_parameter("width", 0);
      is_passed_width = width_ > 0;

      nextParameterToDeclare = "height";
      height_ = this->declare_parameter("height", 0);
      is_passed_height = height_ > 0;

      nextParameterToDeclare = "gain";
      gain_ = this->declare_parameter("gain", -1.0);
      is_passed_gain_ = gain_ >= 0;

      nextParameterToDeclare = "exposure_time";
      exposure_time_ = this->declare_parameter("exposure_time", -1.0);
      is_passed_exposure_time_ = exposure_time_ >= 0;

      nextParameterToDeclare = "trigger_mode";
      trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);

      topic_ = this->declare_parameter(
          "topic", std::string("/") + this->get_name() + "/images");

    } catch (rclcpp::ParameterTypeException& e) {
      log_err(nextParameterToDeclare + " argument");
      throw;
    }

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
    // TODO replace the simple qos with this
    /*
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
    ));
    // The reliability policy can be reliable, meaning that the underlying
    transport layer will try
    // ensure that every message gets received in order, or best effort, meaning
    that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(reliability_policy_)
    */
    m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string(), qos);

    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }
  ~ArenaCameraNode() {}

  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg); };

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srv_;

  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  void wait_for_device_timer_callback_();
  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile();
  void set_nodes_roi();
  void set_nodes_gain();
  void set_nodes_pixelformat();
  void set_nodes_exposure();
  void set_nodes_trigger_mode();
  void publish_images_();

  void publish_an_image_on_trigger_(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  sensor_msgs::msg::Image msg_form_image_(Arena::IImage* pImage);
};
