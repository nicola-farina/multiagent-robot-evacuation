#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "clipper.hpp"
#include "clipper_extensions.hpp"
#include "models.hpp"
#include "vgraph.hpp"
#include "dubins.hpp"
#include "utils.hpp"
#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <mutex>

#include <sys/types.h>
#include <sys/wait.h>

#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"

#include "nav2_msgs/action/follow_path.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


#include <cstdlib>
#include <cmath>

#define _USE_MATH_DEFINES

#include <math.h>

#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
using std::placeholders::_1;
using std::placeholders::_2;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("evacuation") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        // Read map borders
        mapSubscriber_ = this->create_subscription<geometry_msgs::msg::Polygon>(
                "map_borders", qos, std::bind(&MinimalPublisher::bordersCallback, this, _1));

        // Read gate
        gateSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "gate_position", qos, std::bind(&MinimalPublisher::gateCallback, this, _1));

        // Read obstacles
        obstaclesSubscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "obstacles", qos, std::bind(&MinimalPublisher::obstaclesCallback, this, _1));

        if (sim) {
            std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer;
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            std::string toFrameRel = "map";

            try {
                rclcpp::Time now = this->get_clock()->now();
                t1 = tf_buffer->lookupTransform(toFrameRel, "shelfino1/base_link", tf2::TimePointZero, 30s);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                            "shelfino1/base_link", ex.what());
                return;
            }

            try {
                rclcpp::Time now = this->get_clock()->now();
                t2 = tf_buffer->lookupTransform(toFrameRel, "shelfino2/base_link", tf2::TimePointZero, 30s);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                            "shelfino2/base_link", ex.what());
                return;
            }

            try {
                rclcpp::Time now = this->get_clock()->now();
                t3 = tf_buffer->lookupTransform(toFrameRel, "shelfino3/base_link", tf2::TimePointZero, 30s);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                            "shelfino3/base_link", ex.what());
                return;
            }
        }
    }

    void planEvacuation() {
        RCLCPP_INFO(this->get_logger(), "All data retrieved!");

        RCLCPP_INFO(this->get_logger(), "Converting obstacles...");
        std::vector<Polygon> obstacles;
        for (const obstacles_msgs::msg::ObstacleMsg &obstacle_msg: obstaclesData.obstacles) {
            std::vector<Point> obstacle_points;
            for (geometry_msgs::msg::Point32 point: obstacle_msg.polygon.points) {
                obstacle_points.emplace_back(point.x, point.y);
            }
            Polygon polygon = Polygon(obstacle_points);
            obstacles.push_back(polygon);
            obstacle_points.clear();
        }
        RCLCPP_INFO(this->get_logger(), "Obstacles converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Converting map...");
        std::vector<Point> map_points;
        for (geometry_msgs::msg::Point32 point: mapData.points) {
            map_points.emplace_back(point.x, point.y);
        }
        Polygon map = Polygon(map_points);
        RCLCPP_INFO(this->get_logger(), "Map converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Converting gate...");
        Pose gate = Pose(gateData.poses[0].position.x, gateData.poses[0].position.y, gateData.poses[0].orientation.z);
        RCLCPP_INFO(this->get_logger(), "Gate converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Converting robots...");
        Robot shelfino1;
        Robot shelfino2;
        Robot shelfino3;
        if (sim) {
            shelfino1 = Robot(Pose{t1.transform.translation.x, t1.transform.translation.y, t1.transform.rotation.z}, 1);
            shelfino2 = Robot(Pose{t2.transform.translation.x, t2.transform.translation.y, t2.transform.rotation.z}, 2);
            shelfino3 = Robot(Pose{t3.transform.translation.x, t3.transform.translation.y, t3.transform.rotation.z}, 3);
        }
        std::vector<Robot> robots = {shelfino1, shelfino2, shelfino3};
        RCLCPP_INFO(this->get_logger(), "Robots converted successfully!");

        // All data collected, prepare roadmap
        environment::Environment env(map, obstacles, robots, gate);

        dubins::Dubins dubins = dubins::Dubins(robotMaxCurvature, 0.005);  // TODO: discretization

        std::vector<Polygon> polygonsForVisgraph;
        std::vector<Polygon> polygonsForDubins;
        std::vector<std::vector<Polygon>> pols = enlargeAndJoinObstacles(env.getObstacles(), robotRadius);
        polygonsForVisgraph = pols[0];
        polygonsForDubins = pols[1];
        polygonsForDubins.push_back(map);

        vgraph::VGraph visGraph = vgraph::VGraph({shelfino1, shelfino2, shelfino3}, polygonsForVisgraph, gate);
        std::vector<int> curves_len;
        std::vector<std::vector<Point>> finalPoints;
        std::vector<Point> robotPoints;
        for (std::vector<Robot>::size_type r = 0; r < robots.size(); r++) {
            Robot robot = robots[r];
            std::vector<Point> path = visGraph.shortestPath(robot.getPosition(), gate.getPosition());
            auto **points = new dubins::DubinsPoint *[path.size()];
            points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, robot.getOrientation());
            for (std::vector<Point>::size_type i = 1; i < path.size() - 1; i++) {
                points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
            }
            points[path.size() - 1] = new dubins::DubinsPoint(path[path.size() - 1].x, path[path.size() - 1].y);
            curves_len.push_back(path.size() - 1);
            dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygonsForDubins,
                                                                   env.getMap());
            if (curves == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Path not found!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Path computed!");
                for (int i = 0; i < curves_len[r]; i++) {
                    std::vector<Point> curvePoint = getPointsFromCurve(curves[i]);
                    robotPoints.insert(robotPoints.end(), curvePoint.begin(), curvePoint.end());
                }
            }
            finalPoints.push_back(robotPoints);
            robotPoints.clear();
        }

        // Coordination task
        std::vector<std::vector<Point>> finalPointsSafe = getPathsWithoutRobotCollisions(finalPoints[0], finalPoints[1], finalPoints[2]);

        // Publish paths
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");
        rclcpp_action::Client<FollowPath>::SharedPtr action_client_1 = rclcpp_action::create_client<FollowPath>(this,
                                                                                                                "shelfino1/follow_path");
        rclcpp_action::Client<FollowPath>::SharedPtr action_client_2 = rclcpp_action::create_client<FollowPath>(this,
                                                                                                                "shelfino2/follow_path");
        rclcpp_action::Client<FollowPath>::SharedPtr action_client_3 = rclcpp_action::create_client<FollowPath>(this,
                                                                                                                "shelfino3/follow_path");
        if (!action_client_1->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server 1 not available after waiting");
            rclcpp::shutdown();
        }
        if (!action_client_2->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server 2 not available after waiting");
            rclcpp::shutdown();
        }
        if (!action_client_3->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server 3 not available after waiting");
            rclcpp::shutdown();
        }

        nav_msgs::msg::Path path1;
        path1.header.frame_id = "map";
        std::vector<geometry_msgs::msg::PoseStamped> posesTemp;
        geometry_msgs::msg::Pose poseTemp;
        geometry_msgs::msg::Point positionTemp;
        geometry_msgs::msg::Quaternion quaternionTemp;
        geometry_msgs::msg::PoseStamped poseStampedTemp;
        for(std::vector<Robot>::size_type i=0; i < finalPointsSafe[0].size(); i++){
            Point point = finalPointsSafe[0][i];
            positionTemp.x = point.x;
            positionTemp.y = point.y;
            positionTemp.z = 0;

            quaternionTemp.x = 0;
            quaternionTemp.y = 0;
            quaternionTemp.z = 0;
            quaternionTemp.w = 0;

            poseTemp.position = positionTemp;
            poseTemp.orientation = quaternionTemp;

            poseStampedTemp.pose = poseTemp;
            poseStampedTemp.header.frame_id = "base_link";

            posesTemp.push_back(poseStampedTemp);
        }
        path1.poses = posesTemp;

        nav_msgs::msg::Path path2;
        std::vector<geometry_msgs::msg::PoseStamped> posesTemp2;
        geometry_msgs::msg::Pose poseTemp2;
        geometry_msgs::msg::Point positionTemp2;
        geometry_msgs::msg::Quaternion quaternionTemp2;
        geometry_msgs::msg::PoseStamped poseStampedTemp2;
        path2.header.frame_id = "map";
        for(std::vector<Robot>::size_type i=0; i < finalPointsSafe[1].size(); i++){
            Point point = finalPointsSafe[1][i];
            positionTemp2.x = point.x;
            positionTemp2.y = point.y;
            positionTemp2.z = 0;

            quaternionTemp2.x = 0;
            quaternionTemp2.y = 0;
            quaternionTemp2.z = 0;
            quaternionTemp2.w = 0;

            poseTemp2.position = positionTemp2;
            poseTemp2.orientation = quaternionTemp2;

            poseStampedTemp2.pose = poseTemp2;
            poseStampedTemp2.header.frame_id = "base_link";

            posesTemp2.push_back(poseStampedTemp2);
        }


        path2.poses = posesTemp2;

        nav_msgs::msg::Path path3;
        std::vector<geometry_msgs::msg::PoseStamped> posesTemp3;
        geometry_msgs::msg::Pose poseTemp3;
        geometry_msgs::msg::Point positionTemp3;
        geometry_msgs::msg::Quaternion quaternionTemp3;
        geometry_msgs::msg::PoseStamped poseStampedTemp3;
        path3.header.frame_id = "map";
        for(std::vector<Robot>::size_type i=0; i < finalPointsSafe[2].size(); i++){
            Point point = finalPointsSafe[2][i];
            positionTemp3.x = point.x;
            positionTemp3.y = point.y;
            positionTemp3.z = 0;

            quaternionTemp3.x = 0;
            quaternionTemp3.y = 0;
            quaternionTemp3.z = 0;
            quaternionTemp3.w = 0;

            poseTemp3.position = positionTemp3;
            poseTemp3.orientation = quaternionTemp3;

            poseStampedTemp3.pose = poseTemp3;
            poseStampedTemp3.header.frame_id = "base_link";

            posesTemp3.push_back(poseStampedTemp3);
        }
        path3.poses = posesTemp3;

        auto goalMsgR1 = FollowPath::Goal();
        goalMsgR1.path = path1;
        goalMsgR1.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino1");

        action_client_1->async_send_goal(goalMsgR1);

        for(Point p: finalPointsSafe[0]){
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino1");

        auto goalMsgR2 = FollowPath::Goal();
        goalMsgR2.path = path2;
        goalMsgR2.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino2");

        for(Point p: finalPointsSafe[1]){
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        action_client_2->async_send_goal(goalMsgR2);
        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino2");

        auto goalMsgR3 = FollowPath::Goal();
        goalMsgR3.path = path3;
        goalMsgR3.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino3");

        for(Point p: finalPointsSafe[2]){
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        action_client_3->async_send_goal(goalMsgR3);
        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino3");
    }

private:
    void gateCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        gateData = *msg;
        gateReceived = true;
        if (!roadmapGenerated) {
            if (mapReceived && obstaclesPositionsReceived && gateReceived) {
                roadmapGenerated = true;
                planEvacuation();
            }
        }
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        obstaclesData = *msg;
        obstaclesPositionsReceived = true;
        if (!roadmapGenerated) {
            if (mapReceived && obstaclesPositionsReceived && gateReceived) {
                roadmapGenerated = true;
                planEvacuation();
            }
        }
    }

    void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
        mapData = *msg;
        mapReceived = true;
        if (!roadmapGenerated) {
            if (mapReceived && obstaclesPositionsReceived && gateReceived) {
                roadmapGenerated = true;
                planEvacuation();
            }
        }
    }

    std::vector<std::vector<Point>> getPathsWithoutRobotCollisions(std::vector<Point> path1, std::vector<Point> path2, std::vector<Point> path3) {
        RCLCPP_INFO(this->get_logger(), "Path1");
        for(Point p: path1){
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }
        RCLCPP_INFO(this->get_logger(), "Path2");
        for(Point p: path2){
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }
        RCLCPP_INFO(this->get_logger(), "Path3");
        for(Point p: path3){
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        bool path1_finished = false;
        bool path2_finished = false;
        bool path3_finished = false;
        int path1_index = 0;
        int path2_index = 0;
        int path3_index = 0;
        bool p1IntersectP2 = false;
        bool p1IntersectP3 = false;
        bool p2IntersectP3 = false;
        std::vector<Point> safePath1;
        std::vector<Point> safePath2;
        std::vector<Point> safePath3;
        while (!path1_finished && !path2_finished && !path3_finished) {
            // Check if path1 in position path1_index intersects with path2 in position path2_index only if path1 is not finished
            if (!path1_finished) {
                Point p1 = path1[path1_index];
                if (!path2_finished) {
                    Point p2 = path2[path2_index];
                    if (!intersect(p1, p2)) {
                        // P1 does not intersect P2. Now, P3 must be checked to consider P1 safe
                        p1IntersectP2 = false;
                        if (!path3_finished) {
                            Point p3 = path3[path3_index];
                            if (!intersect(p1, p3)) {
                                // P1 does not intersect P3. Now, P2 must be checked to consider P1 safe
                                p1IntersectP3 = false;
                            } else {
                                // P1 intersects P3.
                                p1IntersectP3 = true;
                            }
                        } else {
                            // P3 is finished. Now, P2 must be checked to consider P1 safe
                            p1IntersectP3 = false;
                        }
                    } else {
                        // P1 intersects P2.
                        p1IntersectP2 = true;
                    }
                } else {
                    // P2 is finished. Now, P3 must be checked to consider P1 safe
                    p1IntersectP2 = false;
                    if (!path3_finished) {
                        Point p3 = path3[path3_index];
                        if (!intersect(p1, p3)) {
                            // P1 does not intersect P3.
                            p1IntersectP3 = false;
                        } else {
                            // P1 intersects P3.
                            p1IntersectP3 = true;
                        }
                    } else {
                        // P3 is finished. Now, P2 must be checked to consider P1 safe
                        p1IntersectP3 = false;
                    }
                }
            }

            if (!path2_finished) {
                Point p2 = path2[path2_index];
                if (!path3_finished) {
                    Point p3 = path3[path3_index];
                    if (!intersect(p2, p3)) {
                        // P2 does not intersect P3.
                        p2IntersectP3 = false;
                    } else {
                        // P2 intersects P3.
                        p2IntersectP3 = true;
                    }
                } else {
                    // P3 is finished.
                    p2IntersectP3 = false;
                }
            }

            if (!path1_finished) {
                safePath1.push_back(path1[path1_index]);
                path1_index++;
            } else {
                p1IntersectP2 = false;
                p1IntersectP3 = false;
            }
            if (!path2_finished) {
                if (p1IntersectP2) {
                    safePath2.push_back(path2[path2_index - 1]); // It raise exception only if the first points of the paths intersect. It means that they spawn directly with collision.
                } else {
                    safePath2.push_back(path2[path2_index]);
                    path1_index++;
                }
            } else {
                p2IntersectP3 = false;
            }
            if (!path3_finished) {
                if (p1IntersectP3) {
                    safePath3.push_back(path3[path3_index - 1]); // It raise exception only if the first points of the paths intersect. It means that they spawn directly with collision.
                } else if (p2IntersectP3) {
                    safePath3.push_back(path3[path3_index - 1]); // It raise exception only if the first points of the paths intersect. It means that they spawn directly with collision.
                } else {
                    safePath3.push_back(path3[path3_index]);
                    path3_index++;
                }
            }

            if (path1_index == int(path1.size())) {
                path1_finished = true;
            }
            if (path2_index == int(path2.size())) {
                path2_finished = true;
            }
            if (path3_index == int(path3.size())) {
                path3_finished = true;
            }
        }
        std::vector<std::vector<Point>> safePaths;
        safePaths.push_back(safePath1);
        safePaths.push_back(safePath2);
        safePaths.push_back(safePath3);
        return safePaths;
    }

    bool intersect(Point p1, Point p2) {
        // (R0 - R1)^2 <= (x0 - x1)^2 + (y0 - y1)^2 <= (R0 + R1)^2
        double val = std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2);
        return val >= 0 && val <= std::pow((robotRadius * 2), 2);
    }

    Pose circline(double s, double x0, double y0, double th0, double k) {
        Pose p;
        p.x = x0 + s * dubins::sinc(k * s / 2.0) * cos(th0 + k * s / 2);
        p.y = y0 + s * dubins::sinc(k * s / 2.0) * sin(th0 + k * s / 2);
        p.th = dubins::mod2pi(th0 + k * s);
        return p;
    }

    std::vector<Point> getPointsFromArc(dubins::Arc *arc, int npts) {
        std::vector<Point> points;
        Point point;
        Pose pose;

        for (int j = 0; j < npts; j++) {

            double s = arc->L / npts * j;

            pose = circline(s, arc->x0, arc->y0, arc->th0, arc->k);

            point.x = pose.x;
            point.y = pose.y;

            points.push_back(point);
        }

        return points;
    }

    std::vector<Point> getPointsFromCurve(dubins::Curve *curve) {

        std::vector<Point> line1 = getPointsFromArc(curve->a1, 10);
        std::vector<Point> line2 = getPointsFromArc(curve->a2, 30);
        std::vector<Point> line3 = getPointsFromArc(curve->a3, 10);

        std::vector<Point> totLine;
        totLine.insert(totLine.end(), line1.begin(), line1.end());;
        totLine.insert(totLine.end(), line2.begin(), line2.end());;
        totLine.insert(totLine.end(), line3.begin(), line3.end());;
        return totLine;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber_;

    bool sim = true;
    double robotMaxCurvature = 0.6;
    double robotRadius = 0.3;

    bool mapReceived = false;
    bool gateReceived = false;
    bool obstaclesPositionsReceived = false;
    bool roadmapGenerated = false;

    geometry_msgs::msg::Polygon mapData;
    obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
    geometry_msgs::msg::PoseArray gateData;

    geometry_msgs::msg::TransformStamped t1;
    geometry_msgs::msg::TransformStamped t2;
    geometry_msgs::msg::TransformStamped t3;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}