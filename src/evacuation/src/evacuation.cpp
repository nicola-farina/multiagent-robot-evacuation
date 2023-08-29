#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "clipper.hpp"
#include "clipper_extensions.hpp"
#include "environment.hpp"
#include "vgraph.hpp"
#include "dubins.hpp"
#include "utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "robot.hpp"

using namespace std::chrono_literals;
using namespace evacuation;
using std::vector;
using vgraph::VGraph;

class EvacuationNode : public rclcpp::Node {
public:
    EvacuationNode() : Node("evacuation") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        // Setup robots
        shelfino1 = Robot(this, 1, robotRadius);
        shelfino2 = Robot(this, 2, robotRadius);
        shelfino3 = Robot(this, 3, robotRadius);

        // Read map borders
        mapSubscriber = this->create_subscription<geometry_msgs::msg::Polygon>(
                "map_borders", qos, std::bind(&EvacuationNode::bordersCallback, this, std::placeholders::_1));

        // Read gate
        gateSubscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "gate_position", qos, std::bind(&EvacuationNode::gateCallback, this, std::placeholders::_1));

        // Read obstacles
        obstaclesSubscriber = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "obstacles", qos, std::bind(&EvacuationNode::obstaclesCallback, this, std::placeholders::_1));

        // Setup robot transforms
        for (int id=1; id<4; id++) {
            setupLookupTransform(id);
        }
    }

    void planEvacuation() {
        // ========= PREPARE DATA =========
        RCLCPP_INFO(this->get_logger(), "All data retrieved!");

        RCLCPP_INFO(this->get_logger(), "Converting obstacles...");
        vector<Polygon> obstacles;
        for (const obstacles_msgs::msg::ObstacleMsg &obstacle_msg: obstaclesData.obstacles) {
            vector<Point> obstacle_points;
            for (geometry_msgs::msg::Point32 point: obstacle_msg.polygon.points) {
                obstacle_points.emplace_back(point.x, point.y);
            }
            obstacles.emplace_back(obstacle_points);
            obstacle_points.clear();
        }
        RCLCPP_INFO(this->get_logger(), "Obstacles converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Converting map...");
        vector<Point> map_points;
        for (geometry_msgs::msg::Point32 point: mapData.points) {
            map_points.emplace_back(point.x, point.y);
        }
        Polygon map = Polygon(map_points);
        RCLCPP_INFO(this->get_logger(), "Map converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Converting gate...");
        Pose gate = Pose(gateData.poses[0].position.x, gateData.poses[0].position.y, gateData.poses[0].orientation.z);
        RCLCPP_INFO(this->get_logger(), "Gate converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Reading robots initial poses...");
        shelfino1.pose = Pose{shelfino1Transform.transform.translation.x, shelfino1Transform.transform.translation.y, shelfino1Transform.transform.rotation.z};
        shelfino2.pose = Pose{shelfino2Transform.transform.translation.x, shelfino2Transform.transform.translation.y, shelfino2Transform.transform.rotation.z};
        shelfino3.pose = Pose{shelfino3Transform.transform.translation.x, shelfino3Transform.transform.translation.y, shelfino3Transform.transform.rotation.z};
        vector<Robot> robots = {shelfino1, shelfino2, shelfino3};
        RCLCPP_INFO(this->get_logger(), "Robots initial poses read successfully!");

        // ========= PREPARE ROADMAP =========
        Environment env(map, obstacles, robots, gate);

        dubins::Dubins dubins = dubins::Dubins(dubinsMaxCurvature, 0.005);  // TODO: discretization

        vector<Polygon> polygonsForVisgraph;
        vector<Polygon> polygonsForDubins;
        vector<vector<Polygon>> pols = enlargeAndJoinObstacles(env.getObstacles(), robotRadius);
        polygonsForVisgraph = pols[0];
        polygonsForDubins = pols[1];
        polygonsForDubins.push_back(map);

        VGraph visGraph = VGraph({shelfino1, shelfino2, shelfino3}, polygonsForVisgraph, gate);
        vector<int> curves_len;
        vector<vector<Point>> finalPoints;
        vector<Point> robotPoints;
        for (vector<Robot>::size_type r = 0; r < robots.size(); r++) {
            Robot robot = robots[r];
            vector<Point> path = visGraph.shortestPath(robot.getPosition(), gate.position);
            auto **points = new dubins::DubinsPoint *[path.size()];
            points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, robot.getOrientation());
            for (vector<Point>::size_type i = 1; i < path.size() - 1; i++) {
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
                    vector<Point> curvePoint = getPointsFromCurve(curves[i]);
                    robotPoints.insert(robotPoints.end(), curvePoint.begin(), curvePoint.end());
                }
            }
            finalPoints.push_back(robotPoints);
            robotPoints.clear();
        }

        // ========= AVOID ROBOT COLLISIONS =========
        vector<vector<Point>> finalPointsSafe = getPathsWithoutRobotCollisions(finalPoints[0], finalPoints[1], finalPoints[2]);

        // ========= PUBLISH PATHS =========
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");

        // Wait for all action servers to be available
        for (Robot robot : robots) {
            robot.waitForActionServer();
        }

        // Send path to robot

        /*nav_msgs::msg::Path path1;
        path1.header.frame_id = "map";
        vector<geometry_msgs::msg::PoseStamped> posesTemp;
        geometry_msgs::msg::Pose poseTemp;
        geometry_msgs::msg::Point positionTemp;
        geometry_msgs::msg::Quaternion quaternionTemp;
        geometry_msgs::msg::PoseStamped poseStampedTemp;
        for (vector<Robot>::size_type i = 0; i < finalPointsSafe[0].size(); i++) {
            Point point = finalPointsSafe[0][i];
            positionTemp.x = point.x;
            positionTemp.y = point.y;
            positionTemp.z = 0;

            quaternionTemp.x = 0;
            quaternionTemp.y = 0;
            quaternionTemp.z = 0;
            quaternionTemp.w = 1;

            poseTemp.position = positionTemp;
            poseTemp.orientation = quaternionTemp;

            poseStampedTemp.pose = poseTemp;
            poseStampedTemp.header.frame_id = "base_link";

            posesTemp.push_back(poseStampedTemp);
        }
        path1.poses = posesTemp;

        nav_msgs::msg::Path path2;
        vector<geometry_msgs::msg::PoseStamped> posesTemp2;
        geometry_msgs::msg::Pose poseTemp2;
        geometry_msgs::msg::Point positionTemp2;
        geometry_msgs::msg::Quaternion quaternionTemp2;
        geometry_msgs::msg::PoseStamped poseStampedTemp2;
        path2.header.frame_id = "map";
        for (vector<Robot>::size_type i = 0; i < finalPointsSafe[1].size(); i++) {
            Point point = finalPointsSafe[1][i];
            positionTemp2.x = point.x;
            positionTemp2.y = point.y;
            positionTemp2.z = 0;

            quaternionTemp2.x = 0;
            quaternionTemp2.y = 0;
            quaternionTemp2.z = 0;
            quaternionTemp2.w = 1;

            poseTemp2.position = positionTemp2;
            poseTemp2.orientation = quaternionTemp2;

            poseStampedTemp2.pose = poseTemp2;
            poseStampedTemp2.header.frame_id = "base_link";

            posesTemp2.push_back(poseStampedTemp2);
        }


        path2.poses = posesTemp2;

        nav_msgs::msg::Path path3;
        vector<geometry_msgs::msg::PoseStamped> posesTemp3;
        geometry_msgs::msg::Pose poseTemp3;
        geometry_msgs::msg::Point positionTemp3;
        geometry_msgs::msg::Quaternion quaternionTemp3;
        geometry_msgs::msg::PoseStamped poseStampedTemp3;
        path3.header.frame_id = "map";
        for (vector<Robot>::size_type i = 0; i < finalPointsSafe[2].size(); i++) {
            Point point = finalPointsSafe[2][i];
            positionTemp3.x = point.x;
            positionTemp3.y = point.y;
            positionTemp3.z = 0;

            quaternionTemp3.x = 0;
            quaternionTemp3.y = 0;
            quaternionTemp3.z = 0;
            quaternionTemp3.w = 1;

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

        for (Point p: finalPointsSafe[0]) {
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino1");

        auto goalMsgR2 = FollowPath::Goal();
        goalMsgR2.path = path2;
        goalMsgR2.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino2");

        for (Point p: finalPointsSafe[1]) {
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        action_client_2->async_send_goal(goalMsgR2);
        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino2");

        auto goalMsgR3 = FollowPath::Goal();
        goalMsgR3.path = path3;
        goalMsgR3.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino3");

        for (Point p: finalPointsSafe[2]) {
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }

        action_client_3->async_send_goal(goalMsgR3);
        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino3");*/
    }

private:
    void gateCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        gateData = *msg;
        gateReceived = true;
        if (mapReceived && obstaclesPositionsReceived && gateReceived) {
            planEvacuation();
        }
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        obstaclesData = *msg;
        obstaclesPositionsReceived = true;
        if (mapReceived && obstaclesPositionsReceived && gateReceived) {
            planEvacuation();
        }
    }

    void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
        mapData = *msg;
        mapReceived = true;
        if (mapReceived && obstaclesPositionsReceived && gateReceived) {
            planEvacuation();
        }
    }

    double robotRadius = 0.3;
    Robot shelfino1;
    Robot shelfino2;
    Robot shelfino3;
    geometry_msgs::msg::TransformStamped shelfino1Transform;
    geometry_msgs::msg::TransformStamped shelfino2Transform;
    geometry_msgs::msg::TransformStamped shelfino3Transform;

    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscriber;
    geometry_msgs::msg::Polygon mapData;
    bool mapReceived = false;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber;
    obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
    bool obstaclesPositionsReceived = false;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscriber;
    geometry_msgs::msg::PoseArray gateData;
    bool gateReceived = false;

    double dubinsMaxCurvature = 2.0;

    void setupLookupTransform(const int robotId) {
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        std::string robotName = "shelfino" + std::to_string(robotId);
        std::string targetFrame = "map";
        std::string sourceFrame =  robotName + "/base_link";

        try {
            switch (robotId) {
                case 1:
                    shelfino1Transform = tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero, 30s);
                    break;
                case 2:
                    shelfino2Transform = tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero, 30s);
                    break;
                case 3:
                    shelfino3Transform = tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero, 30s);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Can only work with shelfino1, shelfino2 and shelfino3!");
                    rclcpp::shutdown();
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "[%s] Could not transform %s to %s: %s", robotName.c_str(), targetFrame.c_str(), sourceFrame.c_str(), ex.what());
            sleep(1);
            return;
        }
    }

    vector<vector<Point>> getPathsWithoutRobotCollisions(vector<Point> path1, vector<Point> path2, vector<Point> path3) {
        RCLCPP_INFO(this->get_logger(), "Path1");
        for (Point p: path1) {
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }
        RCLCPP_INFO(this->get_logger(), "Path2");
        for (Point p: path2) {
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", p.x, p.y);
        }
        RCLCPP_INFO(this->get_logger(), "Path3");
        for (Point p: path3) {
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
        vector<Point> safePath1;
        vector<Point> safePath2;
        vector<Point> safePath3;
        while (!path1_finished || !path2_finished || !path3_finished) {
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
                    path2_index++;
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
        vector<vector<Point>> safePaths;
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
        p.position.x = x0 + s * dubins::sinc(k * s / 2.0) * cos(th0 + k * s / 2);
        p.position.y = y0 + s * dubins::sinc(k * s / 2.0) * sin(th0 + k * s / 2);
        p.th = dubins::mod2pi(th0 + k * s);
        return p;
    }

    vector<Point> getPointsFromArc(dubins::Arc *arc, int num_points) {
        vector<Point> points;
        Point point;
        Pose pose;

        for (int j = 0; j < num_points; j++) {

            double s = arc->L / num_points * j;

            pose = circline(s, arc->x0, arc->y0, arc->th0, arc->k);

            point.x = pose.position.x;
            point.y = pose.position.y;

            points.push_back(point);
        }

        return points;
    }

    vector<Point> getPointsFromCurve(dubins::Curve *curve) {

        vector<Point> line1 = getPointsFromArc(curve->a1, 100);
        vector<Point> line2 = getPointsFromArc(curve->a2, 100);
        vector<Point> line3 = getPointsFromArc(curve->a3, 100);

        vector<Point> totLine;
        totLine.insert(totLine.end(), line1.begin(), line1.end());
        totLine.insert(totLine.end(), line2.begin(), line2.end());
        totLine.insert(totLine.end(), line3.begin(), line3.end());
        return totLine;
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EvacuationNode>());
    rclcpp::shutdown();
    return 0;
}