#include "rclcpp/rclcpp.hpp"
#include "core/msg/drive_vel.hpp"
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DrivePublisher : public rclcpp::Node {
  private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<core::msg::DriveVel>::SharedPtr publisher;
    rclcpp::QoS qos = rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE).deadline(200ms);
    rclcpp::PublisherOptions publisher_options;

  private:
    void publish_cmds(){

      auto message = core::msg::DriveVel();
      message.linear_vel = 5;
      message.angular_vel = 5;

      publisher->publish(message);
    }
    
  public:
    DrivePublisher() : Node("drive_pub"){
      
      publisher_options.event_callbacks.deadline_callback = [](rclcpp::QOSDeadlineOfferedInfo) -> void{
        std::cout<<"Missed publish deadline"<<std::endl;
      };

      publisher = this->create_publisher<core::msg::DriveVel>("/qos_test", qos, publisher_options);
      timer = this->create_wall_timer(50ms, std::bind(&DrivePublisher::publish_cmds, this)); 

    }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrivePublisher>());
  rclcpp::shutdown();
  return 0;
}
