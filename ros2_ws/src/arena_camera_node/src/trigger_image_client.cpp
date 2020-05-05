

// import rclpy
#include "rclcpp/rclcpp.hpp"

// from std_srvs.srv import Trigger
#include <chrono>
#include "rclcpp/client.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TriggerArenaImageClientNode : public rclcpp::Node
{
 public:
  explicit TriggerArenaImageClientNode(
      const std::string& node_name = "trigger_arena_image_client")
      : Node(node_name)
  {
    // TODO
    // - more generic name to all the trigger to work for many devices
    std::string srvs_name = "/arena_camera_node/trigger_image";

    m_cli_ = this->create_client<std_srvs::srv::Trigger>(srvs_name);

    // wait if the service is not available
    while (!m_cli_->wait_for_service(1s) && rclcpp::ok()) {
      if (!rclcpp::ok()) {
        log_err("Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      } else {
        log_info(srvs_name + " service not available, waiting again... ->");
      }
    };
    // log_debug_('service is available now')
    // log_debug('trigger request object has been created')
  }
  void send_request()
  {
    auto req_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    m_result = m_cli_->async_send_request(req_);
  }

  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger_Response>> m_result;
  void log_info(const std::string& msg)
  {
    RCLCPP_INFO(this->get_logger(), msg);
  };
  void log_err(const std::string& msg)
  {
    RCLCPP_ERROR(this->get_logger(), msg);
  };

 private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_cli_;
};

int main(int argc, char** argv)
{
  // TODO
  // - should take number of images to trigger
  // - add debug logs

  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<TriggerArenaImageClientNode>(
      "trigger_arena_image_client");
  client_node->send_request();
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client_node, client_node->m_result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto response = client_node->m_result.get();
    if (response->success) {
      client_node->log_info(std::string("SUCCESS : ") + response->message);
    } else {
      client_node->log_err(std::string("FAILURE : ") + response->message);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to trigger image; issue with spin");
  }

  rclcpp::shutdown();

  return 0;
}