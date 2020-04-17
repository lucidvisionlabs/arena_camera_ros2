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

class MinimalPublisher : public rclcpp::Node
{
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
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
};

int main(int argc, char* argv[])
{
  try {
    std::cout << "Try block";
    // prepare example
    Arena::ISystem* pSystem = Arena::OpenSystem();
    pSystem->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
    if (deviceInfos.size() == 0) {
      std::cout << "\nNo camera connected\nPress enter to complete\n";
      std::getchar();
      return 0;
    }
    Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);
    // clean up example
    pSystem->DestroyDevice(pDevice);
    Arena::CloseSystem(pSystem);
  } catch (GenICam::GenericException& ge) {
    std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
    return -1;
  } catch (std::exception& ex) {
    std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
    return -1;
  } catch (...) {
    std::cout << "\nUnexpected exception thrown\n";
    return -1;
  }
  std::cout << "PASS" << '\n';
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}