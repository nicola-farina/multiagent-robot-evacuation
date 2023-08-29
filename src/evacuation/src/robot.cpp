//
// Created by nicola on 29/08/23.
//

#include "robot.hpp"

namespace evacuation {

    Robot::Robot(rclcpp::Node *node, int id, double sizeRadius)
            : pose(Pose(0, 0, 0))
            , node(node)
            , id(id)
            , sizeRadius(sizeRadius)
            , name("shelfino" + std::to_string(id))
            , followPathActionClient(rclcpp_action::create_client<nav2_msgs::action::FollowPath>(node, name)) {}

    int Robot::getId() {
        return id;
    }

    double Robot::getSizeRadius() {
        return sizeRadius;
    }

    Point Robot::getPosition() {
        return pose.position;
    }

    double Robot::getOrientation() {
        return pose.th;
    }

    void Robot::waitForActionServer() {
        if (!followPathActionClient->wait_for_action_server()) {
            RCLCPP_ERROR(node->get_logger(), "[%s] Action server follow_path not available!", name.c_str());
            rclcpp::shutdown();
        }
    }


}