//
// Created by luca on 18/08/23.
//

#include "vgraph.hpp"
#include <cmath>
#include <utility>
#include <set>
#include "convex_hull.hpp"
#include <algorithm>

namespace evacuation::vgraph {
    VGraph::VGraph(std::vector<Robot> robots, std::vector<Polygon> obstacles, Pose gate) {
        nodes = std::vector<Point>();
        edges = std::vector<Edge>();

        // Robots initial positions
        for (const Robot &robot: robots) {
            nodes.push_back(robot.pose.position);
        }

        // Gate position
        nodes.push_back(gate.position);

        // Obstacles positions
        for (const Polygon &obstacle: obstacles) {
            for (Point point: obstacle.points) {
                nodes.push_back(point);
            }
        }

        // Compute the edges between robots
        for (std::vector<Robot>::size_type i = 0; i < robots.size(); i++) {
            for (std::vector<Robot>::size_type j = 0; j < robots.size(); j++) {
                if (i < j) {
                    Robot robot = robots[i];
                    Robot otherRobot = robots[j];
                    if (!intersectsObstacles(robot.getPosition(), otherRobot.getPosition(), obstacles)) {
                        double distance = sqrt(pow(robot.getPosition().x - otherRobot.getPosition().x, 2) + pow(robot.getPosition().y - otherRobot.getPosition().y, 2));
                        edges.emplace_back(robot.getPosition(), otherRobot.getPosition(), distance);
                        adj[robot.getPosition()].emplace_back(otherRobot.getPosition(), distance);
                        adj[otherRobot.getPosition()].emplace_back(robot.getPosition(), distance);
                    }
                }
            }
        }

        // Compute the edges between robots and gate
        for (const Robot &robot: robots) {
            if (!intersectsObstacles(robot.pose.position, gate.position, obstacles)) {
                double distance = sqrt(pow(robot.pose.position.x - gate.position.x, 2) + pow(robot.pose.position.y - gate.position.y, 2));
                edges.emplace_back(robot.pose.position, gate.position, distance);
                adj[robot.pose.position].emplace_back(gate.position, distance);
                adj[gate.position].emplace_back(robot.pose.position, distance);
            }
        }

        // Compute the edges between robots and obstacles
        for (const Robot &robot: robots) {
            for (const Polygon &obstacle: obstacles) {
                for (Point point: obstacle.points) {
                    if (!intersectsObstacles(robot.pose.position, point, obstacles)) {
                        double distance = sqrt(pow(robot.pose.position.x - point.x, 2) + pow(robot.pose.position.y - point.y, 2));
                        edges.emplace_back(robot.pose.position, point, distance);
                        adj[robot.pose.position].emplace_back(point, distance);
                        adj[point].emplace_back(robot.pose.position, distance);
                    }
                }
            }
        }

        // Compute the edges of each obstacle
        for (Polygon obstacle: obstacles) {
            for (std::vector<Point> points: obstacle.getPolygonEdges()) {
                Point point = points[0];
                Point otherPoint = points[1];
                double distance = sqrt(pow(point.x - otherPoint.x, 2) + pow(point.y - otherPoint.y, 2));
                edges.emplace_back(point, otherPoint, distance);
                adj[point].emplace_back(otherPoint, distance);
                adj[otherPoint].emplace_back(point, distance);
            }
        }

        // Add edges of convex hull:
        // 1. Compute convex hull
        // 2. Find possible new edges (the obstacle is concave)
        // 3. Add the new edges

        std::vector<Polygon> convexHullPolygons = convex::getConvexHull(obstacles);
        for(std::vector<evacuation::Polygon>::size_type z = 0; z < convexHullPolygons.size(); z++) {
            // The convex hull should not be computed on the map, which is the last obstacle.
            if(z!=convexHullPolygons.size()-1) {
                Polygon originalPolygon = obstacles[z];
                Polygon convexHullPolygon = convexHullPolygons[z];
                if (originalPolygon.points.size() != convexHullPolygon.points.size()) {
                    originalPolygon.points = convex::orderPoints(originalPolygon.points);
                    int pointsNotInConvexHull = 0;
                    for (std::vector<evacuation::Polygon>::size_type j = 0; j < originalPolygon.points.size(); j++) {
                        Point originalPoint = originalPolygon.points[j];
                        Point convexHullPoint = convexHullPolygon.points[j - pointsNotInConvexHull];
                        if (originalPoint.x != convexHullPoint.x || originalPoint.y != convexHullPoint.y) {
                            // This point is not present in the hull.
                            pointsNotInConvexHull += 1;
                            // This edge can be added as connection in the visibility graph.
                            Point point = originalPolygon.points[(j - 1 + originalPolygon.points.size()) %
                                                                 originalPolygon.points.size()];
                            Point otherPoint = originalPolygon.points[(j + 1) %
                                                                      originalPolygon.points.size()];
                            double distance = sqrt(
                                    pow(point.x - otherPoint.x, 2) + pow(point.y - otherPoint.y, 2));
                            edges.emplace_back(point, otherPoint, distance);
                            adj[point].emplace_back(otherPoint, distance);
                            adj[otherPoint].emplace_back(point, distance);
                        }
                    }
                }
            }
        }

        // Compute the edges between obstacles
        for (std::vector<Polygon>::size_type i = 0; i < obstacles.size(); i++) {
            for (std::vector<Polygon>::size_type j = 0; j < obstacles.size(); j++) {
                if (i < j) {
                    Polygon obstacle = obstacles[i];
                    Polygon otherObstacle = obstacles[j];
                    for (Point point: obstacle.points) {
                        for (Point otherPoint: otherObstacle.points) {
                            if (!intersectsObstacles(point, otherPoint, obstacles)) {
                                double distance = sqrt(pow(point.x - otherPoint.x, 2) + pow(point.y - otherPoint.y, 2));
                                edges.emplace_back(point, otherPoint, distance);
                                adj[point].emplace_back(otherPoint, distance);
                                adj[otherPoint].emplace_back(point, distance);
                            }
                        }
                    }
                }
            }
        }

        // Compute the edges between gate and obstacles
        for (const Polygon &obstacle: obstacles) {
            for (Point point: obstacle.points) {
                if (!intersectsObstacles(point, gate.position, obstacles)) {
                    double distance = sqrt(pow(point.x - gate.position.x, 2) + pow(point.y - gate.position.y, 2));
                    edges.emplace_back(point, gate.position, distance);
                    adj[point].emplace_back(gate.position, distance);
                    adj[gate.position].emplace_back(point, distance);
                }
            }
        }
    }

    bool VGraph::intersectsObstacles(Point start, Point end, const std::vector<Polygon> &obstacles) {
        return std::any_of(
                obstacles.begin(), obstacles.end(),
                [&start, &end](const Polygon &obstacle) { return intersectsObstacle(start, end, obstacle); }
        );
    }

    bool VGraph::intersectsObstacle(Point p1, Point p2, Polygon obstacle) {
        for (std::vector<Point> edge: obstacle.getPolygonEdges()) {
            if (p1 == edge[0] || p1 == edge[1] || p2 == edge[0] || p2 == edge[1]) {
                continue;
            }
            if (lineIntersectsSegment(p1, p2, edge[0], edge[1])) {
                return true;
            }
        }
        return false;
    }

    std::vector<Point> VGraph::shortestPath(Point origin, Point destination) {
        std::map<Point, double> dist;
        std::map<Point, Point> prev;

        std::set<std::pair<double, Point>> setVerticesProcessed;

        setVerticesProcessed.insert(std::make_pair(0, origin));
        dist[origin] = 0;

        // Loop until all shortest distances are finalized (or until the destination is found)
        while (!setVerticesProcessed.empty()) {
            // First vertex in the set - the min distance vertex
            std::pair<double, Point> tmp = *(setVerticesProcessed.begin());
            setVerticesProcessed.erase(setVerticesProcessed.begin());

            Point u = tmp.second;

            // If we reached the destination, we can stop
            if (u == destination) {
                break;
            }

            // Loop through all adjacent vertices of the point u
            for (AdjacentPoint e: adj[u]) {
                Point v = e.point;
                // The weight is simply the distance between the points
                double weight = e.distance;

                // If there is the shortest path from v through u (or it's the first one we find)
                if (!dist.count(v) || dist[v] > dist[u] + weight) {
                    // If we already had a distance from v through u, erase it (then we will insert the new one)
                    if (dist.count(v)) {
                        setVerticesProcessed.erase(setVerticesProcessed.find(std::make_pair(dist[v], v)));
                        prev.erase(prev.find(v));
                    }

                    // Update the distance from the point v
                    dist[v] = dist[u] + weight;

                    setVerticesProcessed.insert(std::make_pair(dist[v], v));
                    prev.insert(std::make_pair(v, u));
                }
            }
        }

        // Reconstruct the path using prev. Start from destination
        std::vector<Point> path;
        while (true) {
            path.push_back(destination);
            if (destination == origin) break;
            auto tmp = prev.find(destination);
            if (tmp == prev.end()) {
                return {};
            }
            destination = tmp->second;
        }
        // Reverse the path
        reverse(path.begin(), path.end());

        return path;
    }

    int VGraph::direction(Point p, Point q, Point r) {
        return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    }

    // Checks if two line segments are collinear and overlapping
    bool VGraph::areCollinearAndOverlapping(Point a1, Point b1, Point a2) {
        // Check if the line segments are collinear
        if (direction(a1, b1, a2) == 0) {
            // Check if the line segments overlap
            if (a2.x <= std::max(a1.x, b1.x) && a2.x >= std::min(a1.x, b1.x) && a2.y <= std::max(a1.y, b1.y) && a2.y >= std::min(a1.y, b1.y)) {
                return true;
            }
        }
        return false;
    }

    // The main function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    bool VGraph::lineIntersectsSegment(Point p1, Point q1, Point p2, Point q2) {
        // Compute the directions of the four line segments
        int d1 = direction(p1, q1, p2);
        int d2 = direction(p1, q1, q2);
        int d3 = direction(p2, q2, p1);
        int d4 = direction(p2, q2, q1);

        // Check if the two line segments intersect
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }

        // Check if the line segments are collinear and overlapping
        if (areCollinearAndOverlapping(p1, q1, p2) || areCollinearAndOverlapping(p2, q2, p1)) {
            return true;
        }

        return false;
    }
}
