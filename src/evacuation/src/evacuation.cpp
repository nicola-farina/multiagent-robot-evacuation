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
#include "clipper.hpp"
#include "clipper_extensions.hpp"
#include "environment.hpp"
#include "vgraph.hpp"
#include "dubins.hpp"
#include "utils.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using namespace evacuation;
using std::vector;
using vgraph::VGraph;

class EvacuationNode : public rclcpp::Node {
public:
    EvacuationNode() : Node("evacuation") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        // Read map borders
        mapSubscriber = this->create_subscription<geometry_msgs::msg::Polygon>(
                "map_borders", qos, std::bind(&EvacuationNode::bordersCallback, this, std::placeholders::_1));

        // Read gate
        gateSubscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "gate_position", qos, std::bind(&EvacuationNode::gateCallback, this, std::placeholders::_1));

        // Read obstacles
        obstaclesSubscriber = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "obstacles", qos, std::bind(&EvacuationNode::obstaclesCallback, this, std::placeholders::_1));

        // Setup robots
        shelfino1 = Robot(1);
        shelfino2 = Robot(2);
        shelfino3 = Robot(3);
        robots = {shelfino1, shelfino2, shelfino3};
        for (Robot robot: robots) {
            setupLookupTransform(robot);
            setupFollowPathActionClient(robot);
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
        RCLCPP_INFO(this->get_logger(), "Robots initial poses read successfully!");

        // ========= PREPARE ROADMAP =========
        Environment env(map, obstacles, robots, gate);

        dubins::Dubins dubins = dubins::Dubins(dubinsMaxCurvature, 0.005);  // TODO: discretization

        // Offset obstacles. For visibility graph we use a larger offset (for safety margin)
        vector<Polygon> polygonsForVisgraph;
        vector<Polygon> polygonsForDubins;
        vector<vector<Polygon>> pols = ClipperLibExtensions::enlargeAndJoinObstacles(env.getObstacles(), robotRadius);
        polygonsForVisgraph = pols[0];
        polygonsForDubins = pols[1];
        polygonsForDubins.push_back(map);

        VGraph visGraph = VGraph(robots, polygonsForVisgraph, gate);
        vector<vector<Pose>> paths;
        for (Robot robot: robots) {
            // Compute the shortest path using visibility graph
            vector<Point> path = visGraph.shortestPath(robot.getPosition(), gate.position);
            // Prepare data structure for dubins (only first and last point have orientation)
            auto **points = new dubins::DubinsPoint *[path.size()];
            points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, robot.getOrientation());
            for (vector<Point>::size_type i = 1; i < path.size() - 1; i++) {
                points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
            }
            points[path.size() - 1] = new dubins::DubinsPoint(path[path.size() - 1].x, path[path.size() - 1].y, gate.th);
            // Compute and interpolate dubins curves
            dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygonsForDubins, env.getMap());
            if (curves == nullptr) {
                RCLCPP_INFO(this->get_logger(), "[%s] Path not found!", robot.getName().c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "[%s] Path computed!", robot.getName().c_str());
                paths.push_back(dubins::Dubins::interpolateCurves(curves, path.size() - 1, 100));
            }
        }

        // ========= AVOID ROBOT COLLISIONS =========
        // vector<vector<Point>> finalPointsSafe = getPathsWithoutRobotCollisions(finalPoints[0], finalPoints[1], finalPoints[2]);

        // ========= PUBLISH PATHS =========
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");

        // Wait for all action servers to be available
        for (Robot robot: robots) {
            waitForFollowPathActionServer(robot);
        }

        // Send path to robots
        for (Robot robot: robots) {
            nav_msgs::msg::Path navPath;
            for (Pose p: paths[robot.id - 1]) {
                navPath.poses.push_back(p.toPoseStamped(this->get_clock()->now(), "map"));
            }
            navPath.header.stamp = this->get_clock()->now();
            navPath.header.frame_id = "map";

            auto goalMsg = nav2_msgs::action::FollowPath::Goal();
            goalMsg.path = navPath;
            goalMsg.controller_id = "FollowPath";
            switch (robot.id) {
                case 1:
                    shelfino1FollowPathActionClient->async_send_goal(goalMsg);
                    break;
                case 2:
                    shelfino2FollowPathActionClient->async_send_goal(goalMsg);
                    break;
                case 3:
                    shelfino3FollowPathActionClient->async_send_goal(goalMsg);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Can only work with shelfino1, shelfino2 and shelfino3!");
                    rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "[%s] Goal sent!", robot.getName().c_str());
        }
    }

private:
    double robotRadius = 0.3;
    Robot shelfino1;
    Robot shelfino2;
    Robot shelfino3;
    geometry_msgs::msg::TransformStamped shelfino1Transform;
    geometry_msgs::msg::TransformStamped shelfino2Transform;
    geometry_msgs::msg::TransformStamped shelfino3Transform;
    vector<Robot> robots;

    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr shelfino1FollowPathActionClient;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr shelfino2FollowPathActionClient;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr shelfino3FollowPathActionClient;

    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscriber;
    geometry_msgs::msg::Polygon mapData;
    obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
    geometry_msgs::msg::PoseArray gateData;
    bool mapReceived = false;
    bool obstaclesPositionsReceived = false;
    bool gateReceived = false;

    double dubinsMaxCurvature = 2.0;

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

    void setupLookupTransform(Robot robot) {
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        std::string targetFrame = "map";
        std::string sourceFrame = robot.getName() + "/base_link";

        try {
            switch (robot.id) {
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
            RCLCPP_INFO(this->get_logger(), "[%s] Could not transform %s to %s: %s", robot.getName().c_str(), targetFrame.c_str(), sourceFrame.c_str(), ex.what());
            sleep(1);
            return;
        }
    }

    void setupFollowPathActionClient(Robot robot) {
        std::string actionName = robot.getName() + "/follow_path";

        switch (robot.id) {
            case 1:
                shelfino1FollowPathActionClient = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, actionName);
                break;
            case 2:
                shelfino2FollowPathActionClient = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, actionName);
                break;
            case 3:
                shelfino3FollowPathActionClient = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, actionName);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Can only work with shelfino1, shelfino2 and shelfino3!");
                rclcpp::shutdown();
        }
    }

    void waitForFollowPathActionServer(Robot robot) {
        switch (robot.id) {
            case 1:
                if (!shelfino1FollowPathActionClient->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "[%s] Action server follow_path not available!", robot.getName().c_str());
                    rclcpp::shutdown();
                }
                break;
            case 2:
                if (!shelfino2FollowPathActionClient->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "[%s] Action server follow_path not available!", robot.getName().c_str());
                    rclcpp::shutdown();
                }
                break;
            case 3:
                if (!shelfino3FollowPathActionClient->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "[%s] Action server follow_path not available!", robot.getName().c_str());
                    rclcpp::shutdown();
                }
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Can only work with shelfino1, shelfino2 and shelfino3!");
                rclcpp::shutdown();
        }
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EvacuationNode>());
    rclcpp::shutdown();
    return 0;
}