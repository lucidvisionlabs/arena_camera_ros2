

// import rclpy
#include "rclcpp/rclcpp.hpp"

// from std_srvs.srv import Trigger
#include <chrono>
#include "rclcpp/client.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

// from
// https://github.com/ros2/demos/blob/eloquent/lifecycle/src/lifecycle_service_client.cpp#L81
template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status =
        future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class TriggerArenaImageClientNode : public rclcpp::Node
{
 public:
  explicit TriggerArenaImageClientNode(
      const std::string& node_name = "trigger_arena_image_client")
      : Node(node_name)
  {
    std::string srvs_name = "/arena_camera_node/trigger_image";
    m_cli_ = this->create_client<std_srvs::srv::Trigger>(srvs_name);
    log_warn_("CLIENT CREATED");

    while (!m_cli_->wait_for_service(1s) && rclcpp::ok()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      } else {
        log_info_(srvs_name + " service not available, waiting again... ->");
      }
    };
    // log_debug_('service is available now')
    // log_debug('trigger request object has been created')
  }
  void send_request()
  {
    auto req_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    m_future = m_cli_->async_send_request(req_);
  }

  // float m_future;
  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger_Response>> m_future;

 private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_cli_;

  void log_info_(std::string msg) { RCLCPP_INFO(this->get_logger(), msg); };
  void log_warn_(std::string msg) { RCLCPP_WARN(this->get_logger(), msg); };
};

int main(int argc, char** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<TriggerArenaImageClientNode>(
      "trigger_arena_image_client");
  client_node->send_request();
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client_node, client_node->m_future) ==
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto suc = client_node->m_future.get()->success;
    auto msg = client_node->m_future.get()->message;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), msg);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed");
  }

  // rclcpp::shutdown();

  return 0;
}