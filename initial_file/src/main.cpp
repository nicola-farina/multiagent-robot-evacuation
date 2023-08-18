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

    Polygon map = Polygon({Point(0, 0), Point(0, 20), Point(20, 20), Point(20, 0)});
    Polygon obstacle = Polygon({Point(2, 2), Point(2, 4), Point(4, 4), Point(4, 2)});
    Polygon obstacle2 = Polygon({Point(6, 3), Point(6, 5), Point(8, 5), Point(8, 3)});
    std::vector<Polygon> obstacles = {obstacle, obstacle2};
    std::vector<Robot> robots = {Robot(Point(0, 0), 0.5)};
    Point gate = {Point(9, 9)};
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
    vgraph::VGraph visGraph = vgraph::VGraph(env.getRobots(), obstacles, env.getGate());
    std::cout << "Created graph" << std::endl;
    for(vgraph::Edge edge: visGraph.getEdges()) {
        std::cout << "Edge: " << edge.start.position.x << " " << edge.start.position.y;
        std::cout << "-" << edge.end.position.x << " " << edge.end.position.y << std::endl;
//        std::cout << "length: " << edge.weight << std::endl;
    }

    std::cout << "EDGES: " << visGraph.getEdges().size() << std::endl;
    // Find the shortest path for each robot

    // Use the dubins library to check if the path is feasible

    // 1. Define the dubins class
    dubins::Dubins dubins = dubins::Dubins(20, 0.005);


    // 2. Define which points the path must pass through
    dubins::DubinsPoint **points = new dubins::DubinsPoint *[4];
    points[0] = new dubins::DubinsPoint(8, 2, 0);
    for(int i = 1; i < 4; i++) {
        points[i] = new dubins::DubinsPoint(8+i, 2+i);
    }

    // 3. Call the multipointShortestPath method, passing the points as in the class DubinsPoint, the obstacles and the map.
    dubins::Curve **curves = dubins.multipointShortestPath(points, 4, obstacles, map);
    if (curves == nullptr) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
        for(int i = 0; i < 4; i++) {
            delete points[i];
        }
        delete[] points;
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
    }

    // If the path is not feasible, find a new path

    // Use the server-action to send the path
}
