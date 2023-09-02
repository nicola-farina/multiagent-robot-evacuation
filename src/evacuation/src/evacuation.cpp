#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

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
#include "dubins_utils.hpp"
#include "coordination.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "angle_utils.hpp"
#include "convex_hull.hpp"

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

        // Setup transforms
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        std::string targetFrame = "map";
        std::string sourceFrame;

        try {
            sourceFrame = "shelfino1/base_link";
            rclcpp::Time now = this->get_clock()->now();
            shelfino1Transform = tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero, 30s);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "[shelfino1] Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
            return;
        }

        try {
            sourceFrame = "shelfino2/base_link";
            rclcpp::Time now = this->get_clock()->now();
            shelfino2Transform = tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero, 30s);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "[shelfino2] Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
            return;
        }

        try {
            sourceFrame = "shelfino3/base_link";
            rclcpp::Time now = this->get_clock()->now();
            shelfino3Transform = tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero, 30s);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "[shelfino3] Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
            return;
        }

        // Setup FollowPath action clients
        shelfino1FollowPathActionClient = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "shelfino1/follow_path");
        shelfino2FollowPathActionClient = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "shelfino2/follow_path");
        shelfino3FollowPathActionClient = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "shelfino3/follow_path");

        // Setup plan publishers
        shelfino1PlanPublisher = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan", 10);
        shelfino2PlanPublisher = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan", 10);
        shelfino3PlanPublisher = this->create_publisher<nav_msgs::msg::Path>("shelfino3/plan", 10);

        // Setup robots
        shelfino1 = Robot(1);
        shelfino2 = Robot(2);
        shelfino3 = Robot(3);
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
        Pose gate = Pose(gateData.poses[0].position.x, gateData.poses[0].position.y, angle::quaternionMsgToYaw(gateData.poses[0].orientation));
        RCLCPP_INFO(this->get_logger(), "Gate converted successfully!");

        RCLCPP_INFO(this->get_logger(), "Reading robots initial poses...");
        shelfino1.pose = Pose{shelfino1Transform.transform.translation.x, shelfino1Transform.transform.translation.y, angle::quaternionMsgToYaw(shelfino1Transform.transform.rotation)};
        shelfino2.pose = Pose{shelfino2Transform.transform.translation.x, shelfino2Transform.transform.translation.y, angle::quaternionMsgToYaw(shelfino2Transform.transform.rotation)};
        shelfino3.pose = Pose{shelfino3Transform.transform.translation.x, shelfino3Transform.transform.translation.y, angle::quaternionMsgToYaw(shelfino3Transform.transform.rotation)};
        vector<Robot> robots = {shelfino1, shelfino2, shelfino3};
        RCLCPP_INFO(this->get_logger(), "Robots initial poses read successfully!");

        // ========= PREPARE ROADMAP =========
        Environment env(map, obstacles, robots, gate);
        dubins::Dubins dubins = dubins::Dubins(dubinsMaxCurvature, 0.005);

        // Offset obstacles. For visibility graph we use a larger offset (for safety margin)
        vector<Polygon> polygonsForVisgraph;
        vector<Polygon> polygonsForDubins;
        vector<vector<Polygon>> pols = ClipperLibExtensions::enlargeAndJoinObstacles(env.getObstacles(), robotRadius);
        polygonsForVisgraph = pols[0];
        polygonsForDubins = pols[1];

        // Offset map. For visibility graph we use a larger offset (for safety margin)
        Polygon mapForVisgraph, mapForDubins;
        vector<Polygon> maps = ClipperLibExtensions::enlargeObstaclesWithTwoOffsets(map, -robotRadius);
        mapForVisgraph = maps[0];
        mapForDubins = maps[1];
        polygonsForDubins.push_back(mapForDubins);
        polygonsForVisgraph.push_back(mapForVisgraph);

        RCLCPP_INFO(this->get_logger(), "Computing visibility graph...");
        VGraph visGraph(robots, polygonsForVisgraph, gate);
        vector<vector<coordination::PoseForCoordination>> paths;
        for (Robot robot: robots) {
            // Compute the shortest path using visibility graph
            RCLCPP_INFO(this->get_logger(), "Computing shortest path...");
            vector<Point> path = visGraph.shortestPath(robot.getPosition(), gate.position);
            if (path.empty()) {
                RCLCPP_INFO(this->get_logger(), "[%s] Path not found!", robot.getName().c_str());
                rclcpp::shutdown();
            }
            // Prepare data structure for dubins (only first and last point have orientation)
            auto **points = new dubins::DubinsPoint *[path.size()];
            points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, robot.getOrientation());
            for (vector<Point>::size_type i = 1; i < path.size() - 1; i++) {
                points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
            }
            points[path.size() - 1] = new dubins::DubinsPoint(path[path.size() - 1].x, path[path.size() - 1].y, gate.th);
            // Compute and interpolate dubins curves
            RCLCPP_INFO(this->get_logger(), "Executing multipoint Dubins...");
            dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygonsForDubins, env.getMap());
            if (curves == nullptr) {
                RCLCPP_INFO(this->get_logger(), "[%s] Multipoint Dubins could not find a path!", robot.getName().c_str());
                rclcpp::shutdown();
            } else {
                RCLCPP_INFO(this->get_logger(), "[%s] Path computed!", robot.getName().c_str());
                paths.push_back(dubins::Dubins::interpolateCurves(curves, path.size() - 1, 100));
            }
            delete[] points;
        }

        // ========= AVOID ROBOT COLLISIONS =========
        RCLCPP_INFO(this->get_logger(), "Coordinating evacuation...");
        vector<coordination::RobotCoordination> safePaths = coordination::getPathsWithoutRobotCollisions(paths[0], paths[1], paths[2], robotRadius);

        // ========= PUBLISH AND FOLLOW PATHS =========
        // Wait for all action servers to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for action servers...");
        waitForFollowPathActionServers();
        RCLCPP_INFO(this->get_logger(), "Action servers available!");

        // Send path to robots
        for (Robot robot: robots) {
            RCLCPP_INFO(this->get_logger(), "[%s] Sending goal...", robot.getName().c_str());
            nav_msgs::msg::Path navPath;
            coordination::RobotCoordination robotCoordination = safePaths[robot.id - 1];
            sleep((int) robotCoordination.timeToWait);
            for (Pose p: robotCoordination.path) {
                navPath.poses.push_back(p.toPoseStamped(this->get_clock()->now(), "map"));
            }
            navPath.header.stamp = this->get_clock()->now();
            navPath.header.frame_id = "map";

            auto goalMsg = nav2_msgs::action::FollowPath::Goal();
            goalMsg.path = navPath;
            goalMsg.controller_id = "FollowPath";
            switch (robot.id) {
                case 1:
                    shelfino1PlanPublisher->publish(navPath);
                    shelfino1FollowPathActionClient->async_send_goal(goalMsg);
                    break;
                case 2:
                    shelfino2PlanPublisher->publish(navPath);
                    shelfino2FollowPathActionClient->async_send_goal(goalMsg);
                    break;
                case 3:
                    shelfino3PlanPublisher->publish(navPath);
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

    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr shelfino1FollowPathActionClient;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr shelfino2FollowPathActionClient;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr shelfino3FollowPathActionClient;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino1PlanPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino2PlanPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino3PlanPublisher;

    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscriber;
    geometry_msgs::msg::Polygon mapData;
    obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
    geometry_msgs::msg::PoseArray gateData;
    bool mapReceived = false;
    bool obstaclesPositionsReceived = false;
    bool gateReceived = false;
    bool evacuationStarted = false;

    double dubinsMaxCurvature = 2.0;

    void gateCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        gateData = *msg;
        gateReceived = true;
        if (!evacuationStarted && mapReceived && obstaclesPositionsReceived && gateReceived) {
            evacuationStarted = true;
            planEvacuation();
        }
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        obstaclesData = *msg;
        obstaclesPositionsReceived = true;
        if (!evacuationStarted && mapReceived && obstaclesPositionsReceived && gateReceived) {
            evacuationStarted = true;
            planEvacuation();
        }
    }

    void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
        mapData = *msg;
        mapReceived = true;
        if (!evacuationStarted && mapReceived && obstaclesPositionsReceived && gateReceived) {
            evacuationStarted = true;
            planEvacuation();
        }
    }

    void waitForFollowPathActionServers() {
        if (!shelfino1FollowPathActionClient->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "[shelfino1] Action server follow_path not available!");
            rclcpp::shutdown();
        }
        if (!shelfino2FollowPathActionClient->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "[shelfino2] Action server follow_path not available!");
            rclcpp::shutdown();
        }
        if (!shelfino3FollowPathActionClient->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "[shelfino3] Action server follow_path not available!");
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
