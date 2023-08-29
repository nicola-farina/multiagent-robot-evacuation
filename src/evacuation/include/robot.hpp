//
// Created by nicola on 29/08/23.
//

#ifndef EVACUATION_ROBOT_HPP
#define EVACUATION_ROBOT_HPP

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "environment.hpp"

namespace evacuation {

    class Robot {
    public:
        Pose pose;

        Robot(rclcpp::Node *node, int id, double sizeRadius);

        int getId();

        double getSizeRadius();

        Point getPosition();

        double getOrientation();

        void waitForActionServer();

    private:
        rclcpp::Node *node;
        int id;
        double sizeRadius;
        std::string name;
        rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr followPathActionClient;
    };

}

#endif
