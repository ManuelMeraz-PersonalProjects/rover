// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

class TimestampedKeyTeleopNode : public rclcpp::Node
{
 public:
   TimestampedKeyTeleopNode() : Node("timestamped_key_vel_publisher")
   {
      m_key_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
         "/key_vel", 10, [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
            const auto& logger = get_logger();
            if (msg == nullptr) {
               RCLCPP_ERROR(logger, "Received key vel message that's a nullptr");
               return;
            }

            auto stamped_twist = geometry_msgs::msg::TwistStamped();
            stamped_twist.twist = *msg;
            stamped_twist.header.stamp = get_clock()->now();
            m_stamped_key_vel_publisher->publish(stamped_twist);
         });

      m_stamped_key_vel_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(
         "/diff_drive_controller/cmd_vel", rclcpp::SystemDefaultsQoS());
   }

 private:
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_key_vel_subscriber;
   rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_stamped_key_vel_publisher;
};

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<TimestampedKeyTeleopNode>());
   rclcpp::shutdown();
   return 0;
}
