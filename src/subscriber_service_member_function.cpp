#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
using std::placeholders::_1;
 
class PublishingSubscriber : public rclcpp::Node
{
  public:
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "input_int", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Int64>("doubled_int",10);  
    }
 
  private:
    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) const
    {
      auto message = std_msgs::msg::Int64();
      message.data = 2 * msg->data;
      RCLCPP_INFO(this->get_logger(), "Received: '%d'; Sending doubled: '%d'", msg->data, message.data);
      publisher_->publish(message);
    }
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}