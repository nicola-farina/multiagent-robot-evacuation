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
#include "dubins.hpp"
#include "utils.hpp"
#include <vector>
#include <chrono>


bool mapReceived = false;
bool gateReceived = false;
bool obstaclesPositionsReceived = false;
bool roadmapReceived = false;

geometry_msgs::msg::Polygon mapData;
obstacles_msgs::msg::ObstacleArrayMsg obstaclesData;
geometry_msgs::msg::PoseArray gateData;

void roadmapCallback(const roadmap_msgs::msg::RoadmapMsg::SharedPtr msg) {
    if (!roadmapReceived && mapReceived && gateReceived && obstaclesPositionsReceived) {
        roadmapReceived = true;
        roadmap_msgs::msg::RoadmapMsg roadmapData = *msg;
        RCLCPP_INFO(rclcpp::get_logger("Subscriber"), "Roadmap received!");
        // Re-build the roadmap.
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
        std::vector<Robot> robots;

        std::vector<Point> map_points;
        for (geometry_msgs::msg::Point32 point: mapData.points) {
            map_points.push_back(Point(point.x, point.y));
        }
        Polygon map = Polygon(map_points);

        std::vector<Point> gates;
        for (geometry_msgs::msg::Pose gate_pose: gateData.poses) {
            gates.push_back(Point(gate_pose.position.x, gate_pose.position.y));
        }

        environment::Environment env(map, obstacles, robots, gates[0]);
        double offset = 0.5;
        std::vector<Polygon> polygons;
        std::vector<std::vector<Polygon>> pols = enlargeAndJoinObstacles(env.getObstacles(), offset);
        polygons = pols[1];

        vgraph::VGraph visGraph = vgraph::VGraph();
        for (const roadmap_msgs::msg::NodeMsg &node_msg: roadmapData.nodes) {
            visGraph.addNode(vgraph::Node(Point(node_msg.x, node_msg.y)));
        }
        for (const roadmap_msgs::msg::EdgeMsg &edge_msg: roadmapData.edges) {
            visGraph.addEdge(
                    vgraph::Edge(Point(edge_msg.node1.x, edge_msg.node1.y), Point(edge_msg.node2.x, edge_msg.node2.y),
                                 edge_msg.weight));
        }
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
            dubins::Dubins dubins = dubins::Dubins(0.7, 0.005);  // TODO: max curvature and discretization.
            dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygons, env.getMap());
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
}
// Define the subscriber callback function
void bordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
    mapData = *msg;
    mapReceived = true;
}

void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
    obstaclesData = *msg;
    obstaclesPositionsReceived = true;
}

void gateCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    gateData = *msg;
    gateReceived = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("planner_node");
    // Create the subscriber
    rclcpp::QoS roadmap_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto roadmap_subscriber = node->create_subscription<roadmap_msgs::msg::RoadmapMsg>("/roadmap", roadmap_qos, roadmapCallback); // Adjust the queue size (10) as needed
    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto map_subscriber = node->create_subscription<geometry_msgs::msg::Polygon>("/map_borders", map_qos, bordersCallback); // Adjust the queue size (10) as needed
    rclcpp::QoS obs_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto obstacles_subscriber = node->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("/obstacles", obs_qos, obstaclesCallback); // Adjust the queue size (10) as needed
    rclcpp::QoS gate_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto gate_subscriber = node->create_subscription<geometry_msgs::msg::PoseArray>("/gate_position", gate_qos, gateCallback); // Adjust the queue size (10) as needed
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
