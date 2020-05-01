

#include <stdexcept>  // std::runtime_err
#include <string>

#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_arena_adapter/pixelformat_translation.h"

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  RCLCPP_WARN(this->get_logger(), "wall timer callback is triggered");

  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    RCLCPP_INFO(this->get_logger(),
                "No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    RCLCPP_INFO(this->get_logger(), "%d arena device(s) has been discoved.",
                device_infos.size());
    run_();
  }
}

void ArenaCameraNode::run_()
{
  RCLCPP_INFO(this->get_logger(), " run()");

  // device -------------------------------------------
  auto device = create_device_ros_();
  m_pDevice.reset(device);

  // nodes --------------------------------------------
  set_nodes_();

  // streaming ----------------------------------------
  m_pDevice->StartStream();

  // publish images -------------------------------
  if (!trigger_mode_activated_) {
    publish_images_();
  }

  RCLCPP_INFO(this->get_logger(), " run() done ");
}
void ArenaCameraNode::publish_images_()
{
  Arena::IImage* image = nullptr;
  while (rclcpp::ok()) {
    try {
      image = m_pDevice->GetImage(100);
      auto image_msg = msg_form_image_(image);
      m_pub_->publish(image_msg);
      RCLCPP_INFO(this->get_logger(),
                  std::string("image ") + std::to_string(image->GetFrameId()) +
                      " published to " +
                      this->get_parameter("topic").as_string());
      this->m_pDevice->RequeueBuffer(image);
    } catch (std::exception& e) {
      if (image) {
        this->m_pDevice->RequeueBuffer(image);
        image = nullptr;
        RCLCPP_WARN(this->get_logger(),
                    std::string("Exception occurred when grabbing an image\n") +
                        e.what());
      }
    }
  };
}

sensor_msgs::msg::Image ArenaCameraNode::msg_form_image_(Arena::IImage* pImage)
{
  try {
    auto image_msg = sensor_msgs::msg::Image();
    auto len =
        pImage->GetWidth() * pImage->GetBitsPerPixel() * pImage->GetHeight();

    image_msg.data.assign(pImage->GetData(), pImage->GetData() + len);

    image_msg.height = pImage->GetHeight();
    image_msg.width = pImage->GetWidth();

    // image_msg.is_bigendian =

    // todo
    // - translate pixel fromat to ros pixelformat string
    // - time stamp
    // - frame id
    return image_msg;
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Failed to create Image ROS MSG");
    return sensor_msgs::msg::Image();
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  log_info("The trigger request is being served");

  /*
  Arena::IImage* image = nullptr;
  try {
   image = m_pDevice->GetImage(100);
   auto image_msg = msg_form_image_(image);
   m_pub_->publish(image_msg);
   RCLCPP_INFO(this->get_logger(),
               std::string("image ") + std::to_string(image->GetFrameId()) +
                   " published to " +
                   this->get_parameter("topic").as_string());
   this->m_pDevice->RequeueBuffer(image);
  } catch (std::exception& e) {
   if (image) {
     this->m_pDevice->RequeueBuffer(image);
     image = nullptr;
     RCLCPP_WARN(this->get_logger(),
                 std::string("Exception occurred when grabbing an image\n") +
                     e.what());
   }
  }*/
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
  // given a serial
  if (!this->get_parameter("serial").as_string().empty()) {
    index = DeviceInfoHelper::get_index_of_serial(
        device_infos, this->get_parameter("serial").as_string());
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  RCLCPP_INFO(this->get_logger(),
              std::string("device created ") +
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
  // max value needs 738 pixels in total to be subtracted from width and height
  // for the max to be published
  auto n = 369;
  auto nodemap = m_pDevice->GetNodeMap();
  auto width_value = this->get_parameter("width").get_value<int64_t>();
  if (width_value > 0) {
    Arena::SetNodeValue<int64_t>(
        nodemap, "Width", width_value);  // TODO is this should be uint64_t
  }
  auto new_w =
      GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Width"))->GetMax() -
      (n * GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Width"))
               ->GetInc());

  Arena::SetNodeValue<int64_t>(nodemap, "Width", new_w);

  auto height_value = this->get_parameter("height").get_value<int64_t>();
  if (height_value > 0) {
    Arena::SetNodeValue<int64_t>(
        nodemap, "Height", height_value);  // TODO is this should be uint64_t
  }
  auto new_h =
      GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Height"))
          ->GetMax() -
      (n * GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Height"))
               ->GetInc());

  Arena::SetNodeValue<int64_t>(nodemap, "Height", new_h);

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(new_w) + "X" +
           std::to_string(new_h));
}

void ArenaCameraNode::set_nodes_gain()
{
  auto nodemap = m_pDevice->GetNodeMap();
  auto gain_value = this->get_parameter("gain").get_value<double>();
  if (gain_value >= 0) {  // not default
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_value);
    log_info(std::string("\tGain set to ") + std::to_string(gain_value));
  }
}
void ArenaCameraNode::set_nodes_pixelformat()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  auto pixelformat_value = this->get_parameter("pixelformat").as_string();
  if (!pixelformat_value.empty()) {
    auto pfnc = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_value];
    if (pfnc.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      /// Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
      //                                     pfnc.c_str());
      // log_info(std::string("PixelFormat set to ") + pfnc);
      log_warn(std::string("\tPixelFormat passed but not set"));

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  }
}

void ArenaCameraNode::set_nodes_exposure()
{
  auto nodemap = m_pDevice->GetNodeMap();
  auto exposure_time_value = this->get_parameter("exposure_time").as_double();
  if (exposure_time_value >= 0) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_value);
  }
}

void ArenaCameraNode::set_nodes_trigger_mode()
{
  auto nodemap = m_pDevice->GetNodeMap();
  auto tl_stream_nodemap = m_pDevice->GetTLStreamNodeMap();
  trigger_mode_activated_ = this->get_parameter("trigger_mode").as_bool();
  if (trigger_mode_activated_) {
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
    log_warn(
        "\ttrigger_mode is activated. No images will be published until images "
        "are requested by trigger_image service");
  }
}