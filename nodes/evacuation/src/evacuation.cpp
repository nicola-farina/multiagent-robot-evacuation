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

        publisherR1_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan", 10);
        publisherR2_ = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan", 10);
        publisherR3_ = this->create_publisher<nav_msgs::msg::Path>("shelfino3/plan", 10);

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

    void resultCallback(const GoalHandle::WrappedResult & result){
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Success!!!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                return;
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
        std::vector<int> curves_len;
        std::vector<std::vector<Point>> finalPoints;
        std::vector<Point> robotPoints;
        for (std::vector<Robot>::size_type r = 0; r < robots.size(); r++) {
            Robot robot = robots[r];
            std::vector<Point> path = visGraph.shortestPath(robot.shape, gates[0]);
            dubins::DubinsPoint **points = new dubins::DubinsPoint *[path.size()];
            points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, robot.radius);
            for (std::vector<Point>::size_type i = 1; i < path.size() - 1; i++) {
                points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
            }
            points[path.size() - 1] = new dubins::DubinsPoint(path[path.size() - 1].x, path[path.size() - 1].y);
            curves_len.push_back(path.size() - 1);
            dubins::Dubins dubins = dubins::Dubins(radius, 0.005);  // TODO: discretization.
            dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygonsForDubins,
                                                                   env.getMap());
            if (curves == nullptr) {
                RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Path not found!");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Path computed!");
                for (int i = 0; i < curves_len[r]; i++){
                    std::vector<Point> curvePoint = getPointsFromCurve(curves[i]);
                    robotPoints.insert(robotPoints.end(), curvePoint.begin(), curvePoint.end());
                }
            }
            finalPoints.push_back(robotPoints);
            robotPoints.clear();
        }
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Publishing paths to shelfino2/follow_path...");
        rclcpp_action::Client<FollowPath>::SharedPtr client_ptrR1 = rclcpp_action::create_client<FollowPath>(this,"shelfino1/follow_path");
        rclcpp_action::Client<FollowPath>::SharedPtr client_ptrR2 = rclcpp_action::create_client<FollowPath>(this,"shelfino2/follow_path");
        rclcpp_action::Client<FollowPath>::SharedPtr client_ptrR3 = rclcpp_action::create_client<FollowPath>(this,"shelfino3/follow_path");
        if (!client_ptrR1->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        if (!client_ptrR2->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        if (!client_ptrR3->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Aoooo");
        nav_msgs::msg::Path path1;
//        path1.header.stamp = this->get_clock()->now();
        path1.header.frame_id = "map";
        std::vector<geometry_msgs::msg::PoseStamped> posesTemp;
        geometry_msgs::msg::Pose poseTemp;
        geometry_msgs::msg::Point positionTemp;
        geometry_msgs::msg::Quaternion quaternionTemp;
        geometry_msgs::msg::PoseStamped poseStampedTemp;
//        for(std::vector<Robot>::size_type i=0; i < finalPoints[0].size(); i++){
        for(std::vector<Robot>::size_type i=0; i < 10; i++){
//            Point point = finalPoints[0][i];
//            positionTemp.x = point.x;
//            positionTemp.y = point.y;
            positionTemp.z = 0;
            positionTemp.x = -4 + (i+1)*0.5;
            positionTemp.y = -3;

            quaternionTemp.x = 0;
            quaternionTemp.y = 0;
            quaternionTemp.z = 0;
            quaternionTemp.w = 0;

            poseTemp.position = positionTemp;
            poseTemp.orientation = quaternionTemp;

            poseStampedTemp.pose = poseTemp;
//            poseStampedTemp.header.stamp = this->get_clock()->now();
            poseStampedTemp.header.frame_id = "base_link";

            posesTemp.push_back(poseStampedTemp);
        }


        path1.poses = posesTemp;
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "UUUU");

//        publisherR1_->publish(path1);
//        sleep(5);

        nav_msgs::msg::Path path2;
        std::vector<geometry_msgs::msg::PoseStamped> posesTemp2;
        geometry_msgs::msg::Pose poseTemp2;
        geometry_msgs::msg::Point positionTemp2;
        geometry_msgs::msg::Quaternion quaternionTemp2;
        geometry_msgs::msg::PoseStamped poseStampedTemp2;
//        path2.header.stamp = this->get_clock()->now();
        path2.header.frame_id = "map";
//        for(std::vector<Robot>::size_type i=0; i < finalPoints[1].size(); i++){
        for(std::vector<Robot>::size_type i=0; i < 10; i++){
//            Point point = finalPoints[1][i];
//            positionTemp2.x = point.x;
//            positionTemp2.y = point.y;
            positionTemp2.z = 0;
            positionTemp2.x = 0 + (i+1)*0.5;
            positionTemp2.y = -3;

            quaternionTemp2.x = 0;
            quaternionTemp2.y = 0;
            quaternionTemp2.z = 0;
            quaternionTemp2.w = 0;

            poseTemp2.position = positionTemp2;
            poseTemp2.orientation = quaternionTemp2;

            poseStampedTemp2.pose = poseTemp2;
//            poseStampedTemp.header.stamp = this->get_clock()->now();
            poseStampedTemp2.header.frame_id = "base_link";

            posesTemp2.push_back(poseStampedTemp2);
        }


        path2.poses = posesTemp2;
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "UUUU");

//        publisherR2_->publish(path2);
//        sleep(5);

        nav_msgs::msg::Path path3;
        std::vector<geometry_msgs::msg::PoseStamped> posesTemp3;
        geometry_msgs::msg::Pose poseTemp3;
        geometry_msgs::msg::Point positionTemp3;
        geometry_msgs::msg::Quaternion quaternionTemp3;
        geometry_msgs::msg::PoseStamped poseStampedTemp3;
//        path3.header.stamp = this->get_clock()->now();
        path3.header.frame_id = "map";
//        for(std::vector<Robot>::size_type i=0; i < finalPoints[2].size(); i++){
        for(std::vector<Robot>::size_type i=0; i < 10; i++){
//            Point point = finalPoints[2][i];
//            positionTemp3.x = point.x;
//            positionTemp3.y = point.y;
            positionTemp3.z = 0;
            positionTemp3.x = 4 + (i+1)*0.5;
            positionTemp3.y = -3;

            quaternionTemp3.x = 0;
            quaternionTemp3.y = 0;
            quaternionTemp3.z = 0;
            quaternionTemp3.w = 0;

            poseTemp3.position = positionTemp3;
            poseTemp3.orientation = quaternionTemp3;

            poseStampedTemp3.pose = poseTemp3;
//            poseStampedTemp.header.stamp = this->get_clock()->now();
            poseStampedTemp3.header.frame_id = "base_link";

            posesTemp3.push_back(poseStampedTemp3);
        }


        path3.poses = posesTemp3;
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "UUUU");

//        publisherR3_->publish(path3);
//        sleep(5);


        auto goalMsgR1 = FollowPath::Goal();
        goalMsgR1.path = path1;
        goalMsgR1.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino1");

        client_ptrR1->async_send_goal(goalMsgR1);
        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino1");

        auto goalMsgR2 = FollowPath::Goal();
        goalMsgR2.path = path2;
        goalMsgR2.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino2");

        client_ptrR2->async_send_goal(goalMsgR2);
        RCLCPP_INFO(this->get_logger(), "Goal sent shelfino2");

        auto goalMsgR3 = FollowPath::Goal();
        goalMsgR3.path = path3;
        goalMsgR3.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal shelfino3");

        client_ptrR3->async_send_goal(goalMsgR3);
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

        Pose circline(double s, double x0, double y0, double th0, double k){
            Pose p;
            p.x = x0 + s * dubins::sinc(k * s / 2.0) * cos(th0 + k * s / 2);
            p.y = y0 + s * dubins::sinc(k * s / 2.0) * sin(th0 + k * s / 2);
            p.th = dubins::mod2pi(th0 + k * s);
            return p;
        }

        std::vector<Point> getPointsFromArc(dubins::Arc *arc, int npts){

            std::vector<Point> points;
            Point point;
            Pose pose;

            double k = arc->k;
            Pose p = circline(arc->L, arc->x0, arc->y0, arc->th0, arc->k); // TODO: check if this is correct
            double true_x = p.x;
            double true_y = p.y;

            Pose temp = circline(arc->L, arc->x0, arc->y0, arc->th0, arc->k);

            double epsilon = 0.01;

            if((abs(temp.x - true_x)>epsilon) || (abs(temp.y - true_y) > epsilon)){
                arc->k = -k;
            }

            for (int j = 0; j < npts; j++){

                double s = arc->L/npts * j;

                pose = circline(s, arc->x0, arc->y0, arc->th0, arc->k);

                point.x = pose.x;
                point.y = pose.y;

                points.push_back(point);
            }

            return points;
        }

        std::vector<Point> getPointsFromCurve(dubins::Curve *curve){

            std::vector<Point> line1 = getPointsFromArc(curve->a1, 10);
            std::vector<Point> line2 = getPointsFromArc(curve->a2, 30);
            std::vector<Point> line3 = getPointsFromArc(curve->a3, 10);

            std::vector<Point> totLine;
            totLine.insert(totLine.end(), line1.begin(), line1.end());;
            totLine.insert(totLine.end(), line2.begin(), line2.end());;
            totLine.insert(totLine.end(), line3.begin(), line3.end());;
            return totLine;
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

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherR1_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherR2_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherR3_;


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
