//
// Created by luca on 18/08/23.
//

#include "vgraph.hpp"
#include <cmath>

namespace vgraph {
    VGraph::VGraph(std::vector<Robot> robots, std::vector<Polygon> obstacles, Pose gate) {
        nodes = std::vector<Node>();
        edges = std::vector<Edge>();

        // Robots initial positions
        for (Robot robot: robots) {
            nodes.emplace_back(robot.getPosition());
        }

        // Gate position
        nodes.emplace_back(gate.getPosition());

        // Obstacles positions
        for (const Polygon &obstacle: obstacles) {
            for (Point point: obstacle.points) {
                nodes.emplace_back(point);
            }
        }

        // Compute the edges between robots
        for (std::vector<Robot>::size_type i = 0; i < robots.size(); i++) {
            for (std::vector<Robot>::size_type j = 0; j < robots.size(); j++) {
                if (i < j) {
                    Robot robot = robots[i];
                    Robot otherRobot = robots[j];
                    if (!intersectsObstacle(robot.getPosition(), otherRobot.getPosition(), obstacles)) {
                        double distance = sqrt(pow(robot.getPosition().x - otherRobot.getPosition().x, 2) + pow(robot.getPosition().y - otherRobot.getPosition().y, 2));
                        edges.emplace_back(Node(robot.getPosition()), Node(otherRobot.getPosition()), distance);
                        adj[robot.getPosition()].emplace_back(otherRobot.getPosition(), distance);
                        adj[otherRobot.getPosition()].emplace_back(robot.getPosition(), distance);
                    }
                }
            }
        }

        // Compute the edges between robots and gate
        for (Robot robot: robots) {
            if (!intersectsObstacle(robot.getPosition(), gate.getPosition(), obstacles)) {
                double distance = sqrt(pow(robot.getPosition().x - gate.x, 2) + pow(robot.getPosition().y - gate.y, 2));
                edges.emplace_back(Node(robot.getPosition()), Node(gate.getPosition()), distance);
                adj[robot.getPosition()].emplace_back(gate.getPosition(), distance);
                adj[gate.getPosition()].emplace_back(robot.getPosition(), distance);
            }
        }

        // Compute the edges between robots and obstacles
        for (Robot robot: robots) {
            for (const Polygon &obstacle: obstacles) {
                for (Point point: obstacle.points) {
                    if (!intersectsObstacle(robot.getPosition(), point, obstacles)) {
                        double distance = sqrt(pow(robot.getPosition().x - point.x, 2) + pow(robot.getPosition().y - point.y, 2));
                        edges.emplace_back(Node(robot.getPosition()), Node(point), distance);
                        adj[robot.getPosition()].emplace_back(point, distance);
                        adj[point].emplace_back(robot.getPosition(), distance);
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
                edges.emplace_back(Node(point), Node(otherPoint), distance);
                adj[point].emplace_back(otherPoint, distance);
                adj[otherPoint].emplace_back(point, distance);
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
                            if (!intersectsObstacle(point, otherPoint, obstacles)) {
                                double distance = sqrt(pow(point.x - otherPoint.x, 2) + pow(point.y - otherPoint.y, 2));
                                edges.emplace_back(Node(point), Node(otherPoint), distance);
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
                if (!intersectsObstacle(point, gate.getPosition(), obstacles)) {
                    double distance = sqrt(pow(point.x - gate.x, 2) + pow(point.y - gate.y, 2));
                    edges.emplace_back(Node(point), Node(gate.getPosition()), distance);
                    adj[point].emplace_back(gate.getPosition(), distance);
                    adj[gate.getPosition()].emplace_back(point, distance);
                }
            }
        }
    }

    bool VGraph::intersectsObstacle(Point start, Point end, const std::vector<Polygon> &obstacles) {
        for (const Polygon &obstacle: obstacles) {
            if (intersectsLine(start, end, obstacle)) {
                return true;
            }
        }
        return false;
    };


    bool VGraph::intersectsLine(Point p1, Point p2, Polygon obstacle) {
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

    bool VGraph::onSegment(Point p, Point q, Point r) {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;

        return false;
    }

    int VGraph::orientation(Point p, Point q, Point r) {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        int val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0;  // Collinear

        return (val > 0) ? 1 : 2; // Clock or counterclock wise
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
            std::cout << "Point: " << u.x << ", " << u.y << std::endl;

            // If we reached the destination, we can stop
            if (u == destination) {
                break;
            }

            // Loop through all adjacent vertices of the point u
            for (AdjacentNode e: adj[u]) {
                Point v = e.point;
                // The weight is simply the distance between the points
                double weight = e.distance;

                // If there is a shortest path from v through u (or it's the first one we find)
                if (!dist.count(v) || dist[v] > dist[u] + weight) {
                    // If we already had a distance from v through u, erase it (then we will insert the new one)
                    if (dist.count(v)) {
                        setVerticesProcessed.erase(setVerticesProcessed.find(std::make_pair(dist[v], v)));
                        prev.erase(prev.find(v));
                    }

                    // Update the distance from the point v
                    dist[v] = dist[u] + weight;
                    std::cout << "Distance: " << dist[v] << std::endl;

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
            destination = prev.find(destination)->second;
        }
        // Reverse the path
        reverse(path.begin(), path.end());

        return path;
    }

    // The main function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    bool VGraph::lineIntersectsSegment(Point p1, Point q1, Point p2, Point q2) {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }
}
