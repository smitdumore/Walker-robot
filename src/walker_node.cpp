/************************************************************************************
 * Apache License 2.0
 * Copyright (c) 2022, Smit Dumore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    walker_node.cpp
 *  @author  Smit Dumore
 *  @date    12/05/2022
 *  @version 0.1
 *
 *  @brief implementing a coverage planner
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;


/**
 * @brief Class planner 
 * 
 */
class Planner : public rclcpp::Node {

    
    public:
        Planner() : Node("cleaner_node") {
            auto pubTopicName = "cmd_vel";
            publisher_ = this->create_publisher<TWIST>(pubTopicName, 10);

            auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

            auto subTopicName = "/scan";
            auto scanCallback = std::bind(&Planner::scan_callback, this, _1);
            subscription_ = this->create_subscription<SCAN>
                                    (subTopicName, default_qos, scanCallback);

            auto timerCallback = std::bind(&Planner::timer_callback, this);
            timer_ = this->create_wall_timer(100ms, timerCallback);            
        }
    private:

        SCAN laser_;
        StateType state_;

        rclcpp::Subscription<SCAN>::SharedPtr subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<TWIST>::SharedPtr publisher_;


        //member prototypes
        void scan_callback(const SCAN&);
        void timer_callback();
        bool isObstacle();
};

void Planner::scan_callback(const SCAN& scan_msg) {
    laser_ = scan_msg;
}

void Planner::timer_callback() {
    if (laser_.header.stamp.sec == 0)
      return;

    auto message = TWIST();

    switch (state_) {
        case FORWARD:
        if (isObstacle() == true) {
            state_ = STOP;
            message.linear.x = 0.0;
            message.linear.y = 0.0;
            message.linear.z = 0.0;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
        }
        break;

        case STOP:
        if (isObstacle() == true) {
            state_ = TURN;
            message.linear.x = 0.0;
            message.linear.y = 0.0;
            message.angular.z = 0.1;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
        } else {
            state_ = FORWARD;
            message.linear.x = 0.2;
            message.linear.y = 0.0;
            message.linear.z = 0.0;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;

        case TURN:
        if (isObstacle() == false) {
            state_ = FORWARD;
            message.linear.x = 0.2;
            message.linear.y = 0.0;
            message.linear.z = 0.0;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
    }

}

bool Planner::isObstacle() {

    int scan_size = laser_.ranges.size();
    for(int i=0 ; i < scan_size ; i++) {
        if(laser_.ranges[i] <= laser_.range_min || laser_.ranges[i] >= laser_.range_max) {
            //skipping invalid laser scans
            continue;
        }

        if(laser_.ranges[i] < 1.0){
            // if any laser reading is closer than 1 m
            // then its an obstacle
            RCLCPP_INFO_STREAM(this->get_logger(), "Obstacle Present");
            return true;
        }
    }

    return false;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}