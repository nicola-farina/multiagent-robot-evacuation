//
// Created by nicola on 31/08/23.
//

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "angle_utils.hpp"

namespace angle {
    double quaternionMsgToYaw(geometry_msgs::msg::Quaternion quaternionMsg) {
        tf2::Quaternion gateQuaternion;
        gateQuaternion.setX(quaternionMsg.x);
        gateQuaternion.setY(quaternionMsg.y);
        gateQuaternion.setZ(quaternionMsg.z);
        gateQuaternion.setW(quaternionMsg.w);
        gateQuaternion.normalize();
        tf2::Matrix3x3 gateQuaternionMatrix(gateQuaternion);
        double roll, pitch, yaw;
        gateQuaternionMatrix.getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::msg::Quaternion yawToQuaternionMsg(double yaw) {
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);
        quaternion.normalize();

        geometry_msgs::msg::Quaternion msg;
        msg.x = quaternion.getX();
        msg.y = quaternion.getY();
        msg.z = quaternion.getZ();
        msg.w = quaternion.getW();
        return msg;
    }
}