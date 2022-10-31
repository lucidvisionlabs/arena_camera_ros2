#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

void ArenaCameraNode::parse_parameters_()
{
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
    // no need to is_passed_trigger_mode_ because it is already a boolean

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");
    // no need to is_passed_topic_

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
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
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();
  m_pDevice->StartStream();

  if (!trigger_mode_activated_) {
    publish_images_();
  } else {
    // else ros::spin will
  }
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok()) {
    try {
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      pImage = m_pDevice->GetImage(1000);
      msg_form_image_(pImage, *p_image_msg);

      m_pub_->publish(std::move(p_image_msg));

      log_info(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      this->m_pDevice->RequeueBuffer(pImage);

    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        log_warn(std::string("Exception occurred while publishing an image\n") +
                 e.what());
      }
    }
  };
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_trigger_mode_();
  //set_nodes_test_pattern_image_();
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    // Set the trigger source to software in order to trigger buffers
    // without the use of any additional hardware.
    // Lines of the GPIO can also be used to trigger.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           "FrameStart");
    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}
