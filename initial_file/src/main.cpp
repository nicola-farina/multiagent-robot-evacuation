//
// Created by luca on 14/08/23.
//

#include "models.hpp"
#include "clipper_extensions.hpp"
#include <iostream>
#include <vector>
#include "dubins.hpp"
#include "vgraph.hpp"


int main(int argc, char** argv) {

    // Read the robots position (initial position)

    // Read the gates position (final position)

    // Read the obstacles position

    // Read the map size

    Polygon map = Polygon({Point(-100, 100), Point(100, 100), Point(100, -100), Point(-100, -100)});
    Polygon obstacle = Polygon({Point(20, 20), Point(30, 20), Point(30, 10), Point(20, 10)});
//    Polygon obstacle = Polygon({Point(10, 68), Point(80, 68), Point(80, 30), Point(10, 30)});
//    Polygon obstacle2 = Polygon({Point(35, 40), Point(45, 40), Point(45, 60), Point(35, 60)});
//    Polygon obstacle3 = Polygon({Point(20, 40), Point(30, 40), Point(30, 60), Point(20, 60)});
    std::vector<Polygon> obstacles = {obstacle};
    std::vector<Robot> robots = {Robot(Point(25, 0), 0.5)};
    Point gate = {Point(25, 74)};
    environment::Environment env(map, obstacles, robots, gate);

    // Enlarge the obstacles and merge them if they collide
    std::vector<std::vector<Polygon>> enlargedObstacles = enlargeAndJoinObstacles(env.getObstacles(), 1);
    std::cout << "Enlarged obstacles:" << std::endl;
    std::vector<Polygon> enlargedObstaclesForGraph = enlargedObstacles[0];
    std::vector<Polygon> enlargedObstaclesForCollisionDetection = enlargedObstacles[1];

    for(const Polygon& obstacle : obstacles) {
        std::cout << "Original obstacle: ";
        for(const Point& point : obstacle.points) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }

    for(const Polygon& obstacle : enlargedObstaclesForGraph) {
        std::cout << "Obstacle for graph: ";
        for(const Point& point : obstacle.points) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }

    for(const Polygon& obstacle : enlargedObstaclesForCollisionDetection) {
        std::cout << "Obstacle for collision detection: ";
        for(const Point& point : obstacle.points) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }


    // Add borders to the map
    std::vector<Point> mapPoints = enlarge(env.getMap().points, -1);

    std::cout << "Original map: ";
    for(const Point& point: map.points) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;

    std::cout << "Enlarged map: ";
    for(const Point& point: mapPoints) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;

    // Create the graph using an algorithm of your choice
    vgraph::VGraph visGraph = vgraph::VGraph(env.getRobots(), enlargedObstaclesForGraph, env.getGate());
    std::cout << "Created graph" << std::endl;
    for(vgraph::Edge edge: visGraph.getEdges()) {
        std::cout << "Edge: " << edge.start.position.x << " " << edge.start.position.y;
        std::cout << "-" << edge.end.position.x << " " << edge.end.position.y << std::endl;
//        std::cout << "length: " << edge.weight << std::endl;
    }

    std::cout << "EDGES: " << visGraph.getEdges().size() << std::endl;

    std::map<Point, std::vector<vgraph::AdjacentNode>> adj = visGraph.getAdj();

    for(auto it = adj.begin(); it != adj.end(); ++it) {
        std::cout << "Node: " << it->first.x << " " << it->first.y << std::endl;
        std::cout << "Adjacent nodes: " << std::endl;
        for(vgraph::AdjacentNode node : it->second) {
            std::cout << node.point.x << " " << node.point.y << std::endl;
            std::cout << "Distance: " << node.distance << std::endl;
        }
    }

    // Find the shortest path for each robot
    std::vector<Point> shortest = visGraph.shortestPath(robots[0].shape, gate);
    std::cout << "Shortest path len: " << shortest.size() << std::endl;

    for (Point point : shortest) {
        std::cout << "Point: " << point.x << " " << point.y << std::endl;
    }
    // Use the dubins library to check if the path is feasible

    // 1. Define the dubins class
    dubins::Dubins dubins = dubins::Dubins(20, 0.005);


    // 2. Define which points the path must pass through
    std::vector<dubins::DubinsPoint> points;
    for(int i = 0; i < shortest.size(); i++) {
        dubins::DubinsPoint point;
        if(i == 0) {
            point = dubins::DubinsPoint(shortest[i].x, shortest[i].y, robots[0].radius);
        } else {
            point = dubins::DubinsPoint(shortest[i].x, shortest[i].y);
        }
        points.push_back(point);
    }
    std::cout << "Defined points" << std::endl;
    // 3. Call the multipointShortestPath method, passing the points as in the class DubinsPoint, the obstacles and the map.
    std::vector<dubins::Curve> curves = dubins.multipointShortestPath(points, shortest.size(), enlargedObstaclesForCollisionDetection, map);
    std::cout << "Computed path" << std::endl;
    if (curves.size() != shortest.size()) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
//        for(int i = 0; i < 4; i++) {
//            delete points[i];
//        }
//        delete[] points;
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        dubins.printCompletePath(curves, shortest.size()-1, obstacles);
//        for (int i = 0; i < curves[0]->getLength(); i++) {
//            std::cout << "Point: " << curves[0]->a1. << " " << curves[0]->getPoint(i).y << std::endl;
//        }

//        }
    }

    // If the path is not feasible, find a new path

    // Use the server-action to send the path
}
