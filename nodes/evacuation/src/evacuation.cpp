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


//struct RobotPose {
//    double x;
//    double y;
//    double z;
//    double roll;
//    double pitch;
//    double yaw;
//
//    RobotPose(double x, double y, double z, double roll, double pitch, double yaw) : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
//};

//RobotPose* getRobotPose(rclpp::Node node, std::string robot_name) {
//    std::string target_frame_ = node->declare_parameter<std::string>("target_frame", robot_name);
//    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
//    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
//    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//    std::string fromFrameRel = target_frame_.c_str();
//    std::string toFrameRel = "map";
//    geometry_msgs::msg::TransformStamped t;
//    try {
//        t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 5s);
//    } catch (const tf2::TransformException & ex) {
//        RCLCPP_INFO(node->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
//        return;
//    }
//    tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
//    tf2::Matrix3x3 m(q);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);
//    double x = t.transform.translation.x;
//    double y = t.transform.translation.y;
//    double z = t.transform.translation.z;
//    RobotPose *robot_pose = new RobotPose(x, y, z, roll, pitch, yaw);
//    return robot_name;
//}



class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("evacuation") {

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        //TOPIC SUBSCRIPTION TO READ BORDERS
        mapSubscriber_ = this->create_subscription<geometry_msgs::msg::Polygon>(
                "map_borders", qos, std::bind(&MinimalPublisher::bordersCallback, this, _1));

        //TOPIC SUBSCRIPTION TO READ GATE
        gateSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "gate_position", qos, std::bind(&MinimalPublisher::gateCallback, this, _1));

        // //TOPIC SUBSCRIPTION TO READ OBSTACLES
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
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                            "shelfino1/base_link", ex.what());
                return;
            }

            try {
                rclcpp::Time now = this->get_clock()->now();
                t2 = tf_buffer->lookupTransform(toFrameRel, "shelfino2/base_link", tf2::TimePointZero, 30s);
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                            "shelfino2/base_link", ex.what());
                return;
            }

            try {
                rclcpp::Time now = this->get_clock()->now();
                t3 = tf_buffer->lookupTransform(toFrameRel, "shelfino3/base_link", tf2::TimePointZero, 30s);
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                            "shelfino3/base_link", ex.what());
                return;
            }
        }
    }

    void planEvacuation() {
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "All data retrieved!");

        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Converting obstacles...");

        std::vector<Polygon> obstacles;
        // Generate the obstacles using our data structure.
        for (obstacles_msgs::msg::ObstacleMsg obstacle_msg: obstaclesData.obstacles) {
            std::vector<Point> obstacle_points;
            for (geometry_msgs::msg::Point32 point: obstacle_msg.polygon.points) {
                obstacle_points.push_back(Point(point.x, point.y));
            }
            Polygon polygon = Polygon(obstacle_points);
            obstacles.push_back(polygon);
            obstacle_points.clear();
        }
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Obstacles converted successfully!");

        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Converting map...");

        // Generate the map using our data structure.
        std::vector<Point> map_points;
        for (geometry_msgs::msg::Point32 point: mapData.points) {
            map_points.push_back(Point(point.x, point.y));
        }
        Polygon map = Polygon(map_points);

        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Map converted successfully!");

        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Converting gate...");

        // Generate the gate using our data structure.
        std::vector<Point> gates;
        for (geometry_msgs::msg::Pose gate_pose: gateData.poses) {
            gates.push_back(Point(gate_pose.position.x, gate_pose.position.y));
        }

        double radius = 0.6;
        double robotSize = 0.3;

        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Gate converted successfully!");

        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Converting robots...");
        Robot shelfino1;
        Robot shelfino2;
        Robot shelfino3;
        if (sim) {
            shelfino1 = Robot(Point{t1.transform.translation.x, t1.transform.translation.y}, t1.transform.rotation.z);
            shelfino2 = Robot(Point{t2.transform.translation.x, t2.transform.translation.y}, t2.transform.rotation.z);
            shelfino3 = Robot(Point{t3.transform.translation.x, t3.transform.translation.y}, t3.transform.rotation.z);
        } else {
            // TODO
            shelfino1 = Robot(Point{0, 0}, 0);
            shelfino2 = Robot(Point{0, 0}, 0);
            shelfino3 = Robot(Point{0, 0}, 0);
        }
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Robots converted successfully!");

        std::vector<Robot> robots = {shelfino1, shelfino2, shelfino3};

        // Here now I should have all the data I need to start the algorithm.
        environment::Environment env(map, obstacles, robots, gates[0]);
        std::vector<Polygon> polygonsForVisgraph;
        std::vector<Polygon> polygonsForDubins;
        std::vector<std::vector<Polygon>> pols = enlargeAndJoinObstacles(env.getObstacles(), robotSize);
        polygonsForVisgraph = pols[0];
        polygonsForDubins = pols[1];
        std::vector<Point> mapPoints = enlarge(map_points, -1);
        polygonsForDubins.push_back(map);
        vgraph::VGraph visGraph = vgraph::VGraph({shelfino1, shelfino2, shelfino3}, polygonsForVisgraph, gates[0]);
        dubins::Curve ***paths;
        paths = new dubins::Curve **[robots.size()];
        std::vector<int> curves_len;
        for (std::vector<Robot>::size_type r = 0; r < robots.size(); r++) {
            Robot robot = robots[r];
            std::vector<Point> path = visGraph.shortestPath(robot.shape, gates[0]);
            dubins::DubinsPoint **points = new dubins::DubinsPoint *[path.size()];
            points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, 3.14);
            for (std::vector<Point>::size_type i = 1; i < path.size() - 1; i++) {
                points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
            }
            points[path.size() - 1] = new dubins::DubinsPoint(path[path.size() - 1].x, path[path.size() - 1].y);
            curves_len.push_back(path.size() - 1);
            dubins::Dubins dubins = dubins::Dubins(radius, 0.005);  // TODO: discretization.
            dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygonsForDubins,
                                                                   env.getMap());
            paths[r] = curves;
            if (curves == nullptr) {
                RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Path not found!");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Path computed!");
            }
            // TODO
//        publishPaths(paths, curves_len, robots, node);
        }
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
            //    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "obstracles data len: %ld", obstaclesData.obstacles.size());
            //    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "obstracles data len: %ld", obstaclesData.obstacles[0].polygon.points.size());
        }

        // Define the subscriber callback function
        void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
            mapData = *msg;
            mapReceived = true;

            if (!roadmapGenerated) {
                if (mapReceived && obstaclesPositionsReceived && gateReceived) {
                    roadmapGenerated = true;
                    planEvacuation();
                }
            }
            //    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "map data: %ld", mapData.points.size());
            //    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "map data: %f", mapData.points[0].x);
            //    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "map data: %f", mapData.points[0].y);
        }

        double offset = 0.5;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mapSubscriber_;
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber_;

        bool sim = true;

        bool mapReceived = false;
        bool gateReceived = false;
        bool obstaclesPositionsReceived = false;
        bool roadmapGenerated = false;
        //bool shelfino1PositionsReceived = false;
        //bool shelfino2PositionsReceived = false;
        //bool shelfino3PositionsReceived = false;

        geometry_msgs::msg::Polygon mapData;
        obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
        geometry_msgs::msg::PoseArray gateData;

        geometry_msgs::msg::TransformStamped t1;
        geometry_msgs::msg::TransformStamped t2;
        geometry_msgs::msg::TransformStamped t3;

//        Robot shelfino1;
//        Robot shelfino2;
//        Robot shelfino3;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
