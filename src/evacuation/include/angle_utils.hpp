//
// Created by nicola on 31/08/23.
//

#ifndef EVACUATION_ANGLE_UTILS_HPP
#define EVACUATION_ANGLE_UTILS_HPP

#include <geometry_msgs/msg/detail/quaternion__struct.hpp>

namespace angle {
    double quaternionMsgToYaw(geometry_msgs::msg::Quaternion quaternionMsg);

    geometry_msgs::msg::Quaternion yawToQuaternionMsg(double yaw);
}


#endif //EVACUATION_ANGLE_UTILS_HPP
