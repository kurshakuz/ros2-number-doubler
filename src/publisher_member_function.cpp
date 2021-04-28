#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublishingSubscriber : public rclcpp::Node
{
  public:
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "doubled_int", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Int64>("input_int",10);  

      std_msgs::msg::Int64 message;
      std::string inputString;
      std::cout << "Enter number to double: ";
      std::getline(std::cin, inputString);
    
      std::istringstream iss(inputString);
      long num;
      if (!(iss >> num).fail()) {
        message.data = num;
        RCLCPP_INFO(this->get_logger(), "Sent: '%d'", message.data);
        try {
          publisher_->publish(message);
        } catch (const rclcpp::exceptions::RCLError & e) {
          RCLCPP_ERROR(
          this->get_logger(),
          "unexpectedly failed with %s",
          e.what());
        }
      }
      else {
        std::cerr << "Please enter valid number!" << std::endl;
        rclcpp::shutdown();
      }
    }
 
  private:
    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
      rclcpp::shutdown();
    }
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  // rclcpp::shutdown();
  return 0;
}