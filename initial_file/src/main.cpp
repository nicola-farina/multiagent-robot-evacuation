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

//    // Read the robots position (initial position)
//
//    // Read the gates position (final position)
//
//    // Read the obstacles position
//
//    // Read the map size
//
//    Polygon map = Polygon({Point(-100, 100), Point(100, 100), Point(100, -100), Point(-100, -100)});
//    Polygon obstacle = Polygon({Point(20, 20), Point(30, 20), Point(30, 10), Point(20, 10)});
////    Polygon obstacle = Polygon({Point(10, 68), Point(80, 68), Point(80, 30), Point(10, 30)});
////    Polygon obstacle2 = Polygon({Point(35, 40), Point(45, 40), Point(45, 60), Point(35, 60)});
////    Polygon obstacle3 = Polygon({Point(20, 40), Point(30, 40), Point(30, 60), Point(20, 60)});
//    std::vector<Polygon> obstacles = {obstacle};
//    std::vector<Robot> robots = {Robot(Point(25, 0), 0.5)};
//    Point gate = {Point(25, 74)};
//    environment::Environment env(map, obstacles, robots, gate);
//
//    // Enlarge the obstacles and merge them if they collide
//    std::vector<std::vector<Polygon>> enlargedObstacles = enlargeAndJoinObstacles(env.getObstacles(), 1);
//    std::cout << "Enlarged obstacles:" << std::endl;
//    std::vector<Polygon> enlargedObstaclesForGraph = enlargedObstacles[0];
//    std::vector<Polygon> enlargedObstaclesForCollisionDetection = enlargedObstacles[1];
//
//    for(const Polygon& obstacle : obstacles) {
//        std::cout << "Original obstacle: ";
//        for(const Point& point : obstacle.points) {
//            std::cout << "(" << point.x << ", " << point.y << ") ";
//        }
//        std::cout << std::endl;
//    }
//
//    for(const Polygon& obstacle : enlargedObstaclesForGraph) {
//        std::cout << "Obstacle for graph: ";
//        for(const Point& point : obstacle.points) {
//            std::cout << "(" << point.x << ", " << point.y << ") ";
//        }
//        std::cout << std::endl;
//    }
//
//    for(const Polygon& obstacle : enlargedObstaclesForCollisionDetection) {
//        std::cout << "Obstacle for collision detection: ";
//        for(const Point& point : obstacle.points) {
//            std::cout << "(" << point.x << ", " << point.y << ") ";
//        }
//        std::cout << std::endl;
//    }
//
//
//    // Add borders to the map
//    std::vector<Point> mapPoints = enlarge(env.getMap().points, -1);
//
//    std::cout << "Original map: ";
//    for(const Point& point: map.points) {
//        std::cout << "(" << point.x << ", " << point.y << ") ";
//    }
//    std::cout << std::endl;
//
//    std::cout << "Enlarged map: ";
//    for(const Point& point: mapPoints) {
//        std::cout << "(" << point.x << ", " << point.y << ") ";
//    }
//    std::cout << std::endl;
//
//    // Create the graph using an algorithm of your choice
//    vgraph::VGraph visGraph = vgraph::VGraph(env.getRobots(), enlargedObstaclesForGraph, env.getGate());
//    std::cout << "Created graph" << std::endl;
//    for(vgraph::Edge edge: visGraph.getEdges()) {
//        std::cout << "Edge: " << edge.start.position.x << " " << edge.start.position.y;
//        std::cout << "-" << edge.end.position.x << " " << edge.end.position.y << std::endl;
////        std::cout << "length: " << edge.weight << std::endl;
//    }
//
//    std::cout << "EDGES: " << visGraph.getEdges().size() << std::endl;
//
//    std::map<Point, std::vector<vgraph::AdjacentNode>> adj = visGraph.getAdj();
//
//    for(auto it = adj.begin(); it != adj.end(); ++it) {
//        std::cout << "Node: " << it->first.x << " " << it->first.y << std::endl;
//        std::cout << "Adjacent nodes: " << std::endl;
//        for(vgraph::AdjacentNode node : it->second) {
//            std::cout << node.point.x << " " << node.point.y << std::endl;
//            std::cout << "Distance: " << node.distance << std::endl;
//        }
//    }
//
//    // Find the shortest path for each robot
//    std::vector<Point> shortest = visGraph.shortestPath(robots[0].shape, gate);
//    std::cout << "Shortest path len: " << shortest.size() << std::endl;
//
//    for (Point point : shortest) {
//        std::cout << "Point: " << point.x << " " << point.y << std::endl;
//    }
//    // Use the dubins library to check if the path is feasible
//
//    // 1. Define the dubins class
//    dubins::Dubins dubins = dubins::Dubins(20, 0.005);
//
//
//    // 2. Define which points the path must pass through
//    dubins::DubinsPoint **points = new dubins::DubinsPoint *[shortest.size()];
//    points[0] = new dubins::DubinsPoint(shortest[0].x, shortest[0].y, 0);
//    for(int i = 1; i < shortest.size(); i++) {
//        points[i] = new dubins::DubinsPoint(shortest[i].x, shortest[i].y);
//    }
////    points[shortest.size() - 1]->th = -1.86;
//    std::cout << "Defined points" << std::endl;
//    // 3. Call the multipointShortestPath method, passing the points as in the class DubinsPoint, the obstacles and the map.
//    dubins::Curve **curves = dubins.multipointShortestPath(points, shortest.size(), enlargedObstaclesForCollisionDetection, map);
//    std::cout << "Computed path" << std::endl;
//    if (curves == nullptr) {
//        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
////        for(int i = 0; i < 4; i++) {
////            delete points[i];
////        }
////        delete[] points;
//    } else {
//        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
//        dubins.printCompletePath(curves, shortest.size()-1, obstacles);
////        for (int i = 0; i < curves[0]->getLength(); i++) {
////            std::cout << "Point: " << curves[0]->a1. << " " << curves[0]->getPoint(i).y << std::endl;
////        }
//
////        }
//    }

    // If the path is not feasible, find a new path

    // Use the server-action to send the path



    std::vector<double> tTmp;

    std::vector<Polygon> polygons, polygonsForVisgraph, obstacles;
    double offset = 0.3;
    //std::vector<std::vector<visgraph::Point>> outerWalls;
    // std::vector<visgraph::Point> pol1 {visgraph::Point(1.5, 1.5), visgraph::Point(3.5, 1.5), visgraph::Point(5.0, 3.0), visgraph::Point(3.5, 4.5), visgraph::Point(1.5, 4.5), visgraph::Point(0.0, 3.0)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(5.0, 5.0), visgraph::Point(8.0, 5.0), visgraph::Point(8.0, 11.0), visgraph::Point(5.0, 11.0)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(12.0, 1.0), visgraph::Point(14.0, 4.0), visgraph::Point(12.0, 7.0), visgraph::Point(10.0, 4.0)};
    // std::vector<visgraph::Point> pol4 {visgraph::Point(18.0, 3.0), visgraph::Point(23.0, 7.0), visgraph::Point(12.0, 19.0), visgraph::Point(8.0, 14.0)};
    // visgraph::Point origin = visgraph::Point(1.0, 8.0);
    // visgraph::Point destination = visgraph::Point(24.0, 2.0);

    //The first four polygons are the walls
    Polygon wall = Polygon({Point(-5.0, -5.0), Point(5.0, -5.0), Point(5.0, 5.0), Point(-5.0, 5.0)});

    Polygon pol5 = Polygon({Point(-3.6, -1.2), Point(-3.6, -0.2), Point(-2.0, -0.2), Point(-2.0, -1.2)});
    Polygon pol6 = Polygon({Point(3.6, -1.2), Point(3.6, -0.2), Point(2.0, -0.2), Point(2.0, -1.2)});
//    Polygon pol6 = Polygon({Point(3.0, 8.0), Point(3.0, 12.0), Point(22.0, 12.0), Point(22.0, 8.0)});
//    Polygon pol7 = Polygon({Point(7.0, 14.0), Point(7.0, 17.0), Point(10.0, 17.0), Point(10.0, 14.0)});
    Point origin = Point(-4.0, -3.0);
    Robot robot1 = Robot(origin, 3.14/2);
    origin = Point(0.0, -3.0);
    Robot robot2 = Robot(origin, 3.14/2);
    origin = Point(4.0, -3.0);
    Robot robot3 = Robot(origin, 3.14/2);
    Point destination = Point(0.0, 4.8);


    polygons.push_back(pol5);
    polygons.push_back(pol6);
    std::vector<std::vector<Polygon>> pols = enlargeAndJoinObstacles(polygons, offset);

    obstacles.push_back(pol5);
    obstacles.push_back(pol6);


    polygonsForVisgraph = pols[0];
    polygons = pols[1];
    polygons.push_back(wall);

    vgraph::VGraph visGraph = vgraph::VGraph({robot1, robot2, robot3}, polygonsForVisgraph, destination);

    // COMPUTE SHORTEST PATH
    for (Robot robot: {robot1, robot2, robot3}) {
        std::cout << "SHORTEST PATH:\n";
        std::vector<Point> path = visGraph.shortestPath(robot.shape, destination);
        for (int it = 0; it < path.size(); it++)
            std::cout << "Point: " << path[it].x << " " << path[it].y << std::endl;


        //    printGraph(originalGraph.graph, origin, destination, path);
        //    printGraph(originalGraph2.graph, origin, destination, path);
        //    printGraph(g.graph, origin, destination, path);


        // COMPUTE MULTIPOINT DUBINS SHORTEST PATH
        std::cout << "MULTIPOINT SHORTEST PATH TEST\n";
        dubins::DubinsPoint **points = new dubins::DubinsPoint *[path.size()];
        points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, 3.14 / 2);
        for (int i = 1; i < path.size() - 1; i++) {
            points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
        }
        points[path.size() - 1] = new dubins::DubinsPoint(path[path.size() - 1].x, path[path.size() - 1].y);
        dubins::Dubins dubins = dubins::Dubins(0.6, 0.005);
        dubins::Curve **curves = dubins.multipointShortestPath(points, path.size(), polygons, wall);
        if (curves == nullptr) {
            std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
            std::cout << robot.shape.x << "/" << robot.shape.y << std::endl;
        } else {
            std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
            dubins.printCompletePath(curves, path.size() - 1, polygons);
        }
    }
}