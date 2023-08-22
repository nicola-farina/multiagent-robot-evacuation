#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "roadmap_msgs/msg/roadmap_msg.hpp"
#include "clipper.hpp"
#include "clipper_extensions.hpp"
#include "models.hpp"
#include "vgraph.hpp"
#include <vector>
#include <chrono>

bool mapReceived = false;
bool gateReceived = false;
bool obstaclesPositionsReceived = false;
bool roadmapGenerated = false;
rclcpp::Node::SharedPtr node = nullptr;
//bool shelfino1PositionsReceived = false;
//bool shelfino2PositionsReceived = false;
//bool shelfino3PositionsReceived = false;

geometry_msgs::msg::Polygon mapData;
obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
geometry_msgs::msg::PoseArray gateData;
//your_msgs::RobotPositionMsg shelfino1Data;
//your_msgs::RobotPositionMsg shelfino2Data;
//your_msgs::RobotPositionMsg shelfino3Data;

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

void roadmapGeneration() {
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

    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Gate converted successfully!");

    // Here now I should have all the data I need to start the algorithm.
    double offset = 0.5;
    std::vector<Polygon> polygons, polygonsForVisgraph;
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "1");
    std::vector<std::vector<Polygon>> pols = enlargeAndJoinObstacles(obstacles, offset);
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "2");
    polygonsForVisgraph = pols[0];
    polygons = pols[1];
    std::vector<Point> mapPoints = enlarge(map_points, -1);
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "3");
    Polygon mapArena = Polygon(mapPoints);
    polygons.push_back(map);
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "4");
    Robot shelfino1 = Robot(Point{0, 0}, 0);
    Robot shelfino2 = Robot(Point{-1, 0}, 0);
    Robot shelfino3 = Robot(Point{1, 0}, 0);
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "5");
    vgraph::VGraph visGraph = vgraph::VGraph({shelfino1, shelfino2, shelfino3}, polygonsForVisgraph, gates[0]);
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "6");
    // publish visGraph.
    auto roadmap_publisher = node->create_publisher<roadmap_msgs::msg::RoadmapMsg>("roadmap", 10);
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "7");
    // Convert the VGraph to a Roadmap message
    roadmap_msgs::msg::RoadmapMsg roadmap_msg;
    for (const auto& node_graph : visGraph.getNodes()) {
        // Convert VGraph Node to Roadmap Node
        roadmap_msgs::msg::NodeMsg roadmap_node;
        roadmap_node.x = node_graph.position.x;
        roadmap_node.y = node_graph.position.y;
        roadmap_msg.nodes.push_back(roadmap_node);
    }
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "8");
    for (const auto& edge : visGraph.getEdges()) {
        // Convert VGraph Edge to Roadmap Edge
        roadmap_msgs::msg::EdgeMsg roadmap_edge;
        roadmap_msgs::msg::NodeMsg roadmap_node1;
        roadmap_node1.x = edge.start.position.x;
        roadmap_node1.y = edge.start.position.y;

        roadmap_msgs::msg::NodeMsg roadmap_node2;
        roadmap_node2.x = edge.end.position.x;
        roadmap_node2.y = edge.end.position.y;

        roadmap_edge.node1 = roadmap_node1;
        roadmap_edge.node2 = roadmap_node2;
        roadmap_edge.weight = edge.weight;
        roadmap_msg.edges.push_back(roadmap_edge);
    }
    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "9");

// Publish the Roadmap message
    while(1) {
        roadmap_publisher->publish(roadmap_msg);
        usleep(1000000);
    }
}


// Define the subscriber callback function
void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
    mapData = *msg;
    mapReceived = true;

    if(! roadmapGenerated){
        if(mapReceived && obstaclesPositionsReceived && gateReceived){
            roadmapGenerated = true;
            roadmapGeneration();
        }
    }
//    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "map data: %ld", mapData.points.size());
//    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "map data: %f", mapData.points[0].x);
//    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "map data: %f", mapData.points[0].y);
}

void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
    obstaclesData = *msg;
    obstaclesPositionsReceived = true;
    if(! roadmapGenerated){
        if(mapReceived && obstaclesPositionsReceived && gateReceived){
            roadmapGenerated = true;
            roadmapGeneration();
        }
    }
//    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "obstracles data len: %ld", obstaclesData.obstacles.size());
//    RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "obstracles data len: %ld", obstaclesData.obstacles[0].polygon.points.size());
}

void gateCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    gateData = *msg;
    gateReceived = true;
    if(! roadmapGenerated){
        if(mapReceived && obstaclesPositionsReceived && gateReceived){
            roadmapGenerated = true;
            roadmapGeneration();
        }
    }

}

//void shelfino1Callback(const your_msgs::RobotPositionMsg::SharedPtr msg) {
//    shelfino1Data = *msg;
//    shelfino1PositionsReceived = true;
//}

//void shelfino2Callback(const your_msgs::RobotPositionMsg::SharedPtr msg) {
//    shelfino2Data = *msg;
//    shelfino2PositionsReceived = true;
//}

//void shelfino3Callback(const your_msgs::RobotPositionMsg::SharedPtr msg) {
//    shelfino3Data = *msg;
//    shelfino3PositionsReceived = true;
//}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("roadmap_generation");
    // Create the subscriber
    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto map_subscriber = node->create_subscription<geometry_msgs::msg::Polygon>("/map_borders", map_qos, bordersCallback); // Adjust the queue size (10) as needed
    rclcpp::QoS obs_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto obstacles_subscriber = node->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("/obstacles", obs_qos, obstaclesCallback); // Adjust the queue size (10) as needed
    rclcpp::QoS gate_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto gate_subscriber = node->create_subscription<geometry_msgs::msg::PoseArray>("/gate_position", gate_qos, gateCallback); // Adjust the queue size (10) as needed
//    auto shelfino1_subscriber = node->create_subscription<your_msgs::RobotPositionMsg>("/shelfino1_position", 10, shelfino1Callback); // Adjust the queue size (10) as needed
//    auto shelfino2_subscriber = node->create_subscription<your_msgs::RobotPositionMsg>("/shelfino2_position", 10, shelfino2Callback); // Adjust the queue size (10) as needed
//    auto shelfino3_subscriber = node->create_subscription<your_msgs::RobotPositionMsg>("/shelfino3_position", 10, shelfino3Callback); // Adjust the queue size (10) as needed
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
