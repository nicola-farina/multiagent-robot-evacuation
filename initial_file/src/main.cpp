//
// Created by luca on 14/08/23.
//

#include "models.hpp"
#include "clipper_extensions.hpp"


int main(int argc, char** argv) {

    // Read the robots position (initial position)

    // Read the gates position (final position)

    // Read the obstacles position

    // Read the map size

    Polygon map = Polygon({Point(0, 0), Point(0, 10), Point(10, 10), Point(10, 0)});
    Polygon obstacle = Polygon({Point(2, 2), Point(2, 4), Point(4, 4), Point(4, 2)});
    Polygon obstacle2 = Polygon({Point(6.2, 3), Point(6.2, 5), Point(8, 5), Point(8, 3)});
    std::vector<Polygon> obstacles = {obstacle, obstacle2};
    std::vector<Robot> robots = {Robot(Polygon({Point(0, 0), Point(0, 1), Point(1, 1), Point(1, 0)}), 0.5)};
    std::vector<Polygon> gates = {Polygon({Point(9, 9), Point(9, 10), Point(10, 10), Point(10, 9)})};
    environment::Environment env(map, obstacles, robots, gates);

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


    // Find the shortest path for each robot

    // Use the dubins library to check if the path is feasible
    // 1. Define the dubins class
    // 2. Define which points the path must pass through
    // 3. Call the multipointShortestPath method, passing the points as in the class DubinsPoint, the obstacles and the map.

    // If the path is not feasible, find a new path

    // Use the server-action to send the path
}
