#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "core/msg/drive_vel.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DriveSubscriber : public rclcpp::Node{
  private:
  rclcpp::Subscription<core::msg::DriveVel>::SharedPtr subscriber;

  rclcpp::QoS qos = rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE).deadline(200ms);

  rclcpp::SubscriptionOptions subscriber_options;


  private:
  void drive_callback(const core::msg::DriveVel::SharedPtr msg){
    std::cout << msg->linear_vel<< std::endl;
  }

  public:
  DriveSubscriber() : Node("drive_sub"){

  subscriber_options.event_callbacks.deadline_callback = [](rclcpp::QOSDeadlineRequestedInfo) -> void{
    //set drive to 0 here  
    std::cout<<"0" <<std::endl;

   };

    subscriber = this->create_subscription<core::msg::DriveVel>("qos_test", qos, std::bind(&DriveSubscriber::drive_callback, this, _1), subscriber_options);

    
  } 
};


int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveSubscriber>());
  rclcpp::shutdown();

  return 0;
}

