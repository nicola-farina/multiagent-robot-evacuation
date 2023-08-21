#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "roadmap_msgs/msg/roadmap.hpp"
#include "clipper.hpp"
#include "clipper_extensions.hpp"
#include "models.hpp"
#include "vgraph.hpp"
#include <vector>

bool mapReceived = false;
bool gateReceived = false;
bool obstaclesPositionsReceived = false;
//bool shelfino1PositionsReceived = false;
//bool shelfino2PositionsReceived = false;
//bool shelfino3PositionsReceived = false;

geometry_msgs::msg::Polygon mapData;
obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
geometry_msgs::msg::Pose gateData;
//your_msgs::RobotPositionMsg shelfino1Data;
//your_msgs::RobotPositionMsg shelfino2Data;
//your_msgs::RobotPositionMsg shelfino3Data;


// Define the subscriber callback function
void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
    mapData = *msg;
    mapReceived = true;
}

void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
    obstaclesData = *msg;
    obstaclesPositionsReceived = true;
}

void gateCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    gateData = *msg;
    gateReceived = true;
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
    auto node = rclcpp::Node::make_shared("subscriber_node");

    // Create the subscriber
    auto map_subscriber = node->create_subscription<geometry_msgs::msg::Polygon>("/borders", 10, bordersCallback); // Adjust the queue size (10) as needed
    auto obstacles_subscriber = node->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("/obstacles", 10, obstaclesCallback); // Adjust the queue size (10) as needed
    auto gate_subscriber = node->create_subscription<geometry_msgs::msg::Pose>("/gate_position", 10, gateCallback); // Adjust the queue size (10) as needed
//    auto shelfino1_subscriber = node->create_subscription<your_msgs::RobotPositionMsg>("/shelfino1_position", 10, shelfino1Callback); // Adjust the queue size (10) as needed
//    auto shelfino2_subscriber = node->create_subscription<your_msgs::RobotPositionMsg>("/shelfino2_position", 10, shelfino2Callback); // Adjust the queue size (10) as needed
//    auto shelfino3_subscriber = node->create_subscription<your_msgs::RobotPositionMsg>("/shelfino3_position", 10, shelfino3Callback); // Adjust the queue size (10) as needed

    // Loop to wait until all required data is received
    while (!(mapReceived && gateReceived && obstaclesPositionsReceived)) {
        ros::spinOnce();
        ros::Duration(0.1).sleep(); // Sleep for a short duration
    }

    // Here now I should have all the data I need to start the algorithm.
    std::vector<Polygon> polygons, polygonsForVisgraph, obstacles;
    std::vector<std::vector<Polygon>> pols = enlargeAndJoinObstacles(obstacles, offset);
    polygonsForVisgraph = pols[0];
    polygons = pols[1];
    std::vector<Point> mapPoints = enlarge(mapPoints, -1);
    Polygon map = Polygon({mapPoints});
    polygons.push_back(map)

    vgraph::VGraph visGraph = vgraph::VGraph({shelfino1, shelfino2, shelfino3}, polygonsForVisgraph, gate);

    // publish visGraph.
    roadmap_publisher = node->create_publisher<roadmap_msgs::msg::Roadmap>("roadmap", 10);
    // Convert the VGraph to a Roadmap message
    roadmap_msgs::msg::Roadmap roadmap_msg;
    for (const auto& node : vgraph.nodes) {
        // Convert VGraph Node to Roadmap Node
        roadmap_msgs::msg::Node roadmap_node;
        roadmap_node.x = node.position.x;
        roadmap_node.y = node.position.y;
        roadmap_msg.nodes.push_back(roadmap_node);
    }
    for (const auto& edge : vgraph.edges) {
        // Convert VGraph Edge to Roadmap Edge
        your_package_name::msg::Edge roadmap_edge;
        roadmap_msgs::msg::Node roadmap_node1;
        roadmap_node1.x = edge.start.position.x;
        roadmap_node1.y = edge.start.position.y;

        roadmap_msgs::msg::Node roadmap_node2;
        roadmap_node2.x = edge.start.position.x;
        roadmap_node2.y = edge.start.position.y;

        roadmap_edge.node1 = roadmap_node1;
        roadmap_edge.node2 = roadmap_node2;
        roadmap_edge.weight = edge.weight;
        roadmap_msg.edges.push_back(roadmap_edge);
    }

// Publish the Roadmap message
    roadmap_publisher->publish(roadmap_msg);

    return 0;
}
