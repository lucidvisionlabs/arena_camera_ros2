

#include "ArenaCameraNode.h"

#include <stdexcept>  // std::runtime_err
#include <string>

#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_arena_adapter/pixelformat_translation.h"

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
    // else ros::sping will
  }
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* image = nullptr;
  while (rclcpp::ok()) {
    try {
      image = m_pDevice->GetImage(100);
      auto image_msg = msg_form_image_(image);
      m_pub_->publish(image_msg);
      log_info(std::string("image ") + std::to_string(image->GetFrameId()) +
               " published to " + topic_);
      this->m_pDevice->RequeueBuffer(image);
    } catch (std::exception& e) {
      if (image) {
        this->m_pDevice->RequeueBuffer(image);
        image = nullptr;
        log_warn(std::string("Exception occurred when grabbing an image\n") +
                 e.what());
      }
    }
  };
}

sensor_msgs::msg::Image ArenaCameraNode::msg_form_image_(Arena::IImage* pImage)
{
  try {
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    auto image_data_length_in_bytes =
        width_length_in_bytes * pImage->GetHeight();

    auto image_msg = sensor_msgs::msg::Image();
    // for debugging
    image_msg.data = {1, 2, 3, 4};

    // image_msg.data.assign(pImage->GetData(), pImage->GetData() + len);

    image_msg.width = width_;
    image_msg.height = height_;
    image_msg.step = width_length_in_bytes;

    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    image_msg.encoding = pixelformat_ros_;
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);

    return image_msg;
  } catch (...) {
    log_warn("Failed to create Image ROS MSG");
    return sensor_msgs::msg::Image();
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* image = nullptr;
  try {
    // trigger

    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // inifnate loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggring an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    log_debug("getting an image");
    image = m_pDevice->GetImage(100);

    auto msg = std::string("image ") + std::to_string(image->GetFrameId()) +
               " published to " + topic_;
    auto image_msg = msg_form_image_(image);
    m_pub_->publish(image_msg);
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(image);

  } catch (std::exception& e) {
    if (image) {
      this->m_pDevice->RequeueBuffer(image);
      image = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
    // return;
  }

  catch (GenICam::GenericException& e) {
    if (image) {
      this->m_pDevice->RequeueBuffer(image);
      image = nullptr;
    }
    auto msg =
        std::string("Exception occurred whhile grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
    // return;
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
  set_nodes_load_default_profile();
  set_nodes_roi();
  set_nodes_gain();
  set_nodes_pixelformat();
  set_nodes_exposure();
  set_nodes_trigger_mode();
}

void ArenaCameraNode::set_nodes_load_default_profile()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi()
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

void ArenaCameraNode::set_nodes_gain()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void ArenaCameraNode::set_nodes_pixelformat()
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

void ArenaCameraNode::set_nodes_exposure()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
  }
}

void ArenaCameraNode::set_nodes_trigger_mode()
{
  if (trigger_mode_activated_) {
    auto nodemap = m_pDevice->GetNodeMap();
    auto tl_stream_nodemap = m_pDevice->GetTLStreamNodeMap();

    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggerd images by providing proper "
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
    Arena::GetNodeValue<GenICam::gcstring>(tl_stream_nodemap,
                                           "StreamBufferHandlingMode");
    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
}