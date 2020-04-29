

#include <stdexcept>  // std::runtime_err
#include <string>

#include "ArenaCameraNode.h"
#include "LightArena.h"

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
  // if (!(this->trigger_mode_active)) {
  publish_images_();
  //}
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
  RCLCPP_INFO(this->get_logger(), "The trigger request is being served");

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
    index = IDeviceHelpers::get_index_of_serial(
        device_infos, this->get_parameter("serial").as_string());
  }

  return m_pSystem->CreateDevice(device_infos.at(index));
}

void ArenaCameraNode::set_nodes_()
{
  // max value needs 738 pixels in total to be subtracted from width and height
  // for the max to be published
  auto n = 369;

  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");

  auto gain_value = this->get_parameter("gain").get_value<double>();
  if (gain_value != -1) {
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_value);
  }

  auto width_value = this->get_parameter("width").get_value<int64_t>();
  if (width_value != -1) {
    Arena::SetNodeValue<int64_t>(
        nodemap, "Width", width_value);  // TODO is this should be uint64_t
  }
  auto new_w =
      GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Width"))->GetMax() -
      (n * GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Width"))
               ->GetInc());

  Arena::SetNodeValue<int64_t>(nodemap, "Width", new_w);

  auto height_value = this->get_parameter("height").get_value<int64_t>();
  if (height_value != -1) {
    Arena::SetNodeValue<int64_t>(
        nodemap, "Height", height_value);  // TODO is this should be uint64_t
  }
  auto new_h =
      GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Height"))
          ->GetMax() -
      (n * GenApi::CIntegerPtr(m_pDevice->GetNodeMap()->GetNode("Height"))
               ->GetInc());

  Arena::SetNodeValue<int64_t>(nodemap, "Height", new_h);

  // TODO PIXEL FORMAT HANDLEING

  // TODO HANDEL EXPOSURE AUTO and TIME

  // HANDLE TRIGGER MODE
}
