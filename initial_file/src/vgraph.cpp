//
// Created by luca on 18/08/23.
//

#include "vgraph.hpp"
#include <cmath>

namespace vgraph {

    // Define the constructor
    VGraph::VGraph(std::vector<Robot> robots, std::vector<Polygon> obstacles, Point gate) {
        nodes = std::vector<Node>();
        edges = std::vector<Edge>();

        for(Robot robot : robots) {
            Node node = Node(robot.shape);
            nodes.push_back(node);
        }

        Node node = Node(gate);
        nodes.push_back(node);

        for(Polygon obstacle : obstacles) {
            for(Point point : obstacle.points) {
                Node node = Node(point);
                nodes.push_back(node);
            }
        }

//        std::cout << "Nodes size: " << nodes.size() << std::endl;
//        exit(0);

        // Compute the edges between robots
        for (int i = 0; i < robots.size(); i++) {
            for (int j = 0; j < robots.size(); j++) {
                if (i < j) {
                    Robot robot = robots[i];
                    Robot otherRobot = robots[j];
                    if (!intersectsObstacle(robot.shape, otherRobot.shape, obstacles)) {
                        double distance = sqrt(pow(robot.shape.x - otherRobot.shape.x, 2) +
                                               pow(robot.shape.y - otherRobot.shape.y, 2));
                        Edge edge = Edge(robot.shape, otherRobot.shape, distance);
                        edges.push_back(edge);
                        // push in the adj map the robot shape and the other robot shape
                        AdjacentNode adj_node = AdjacentNode(otherRobot.shape, distance);
                        adj[robot.shape].push_back(adj_node);

                        adj_node = AdjacentNode(robot.shape, distance);
                        adj[otherRobot.shape].push_back(adj_node);
                    }
                }
            }
        }

        // Compute the edges between robots and gate
        for(Robot robot: robots) {
            if(!intersectsObstacle(robot.shape, gate, obstacles)) {
                double distance = sqrt(pow(robot.shape.x - gate.x, 2) + pow(robot.shape.y - gate.y, 2));
                Edge edge = Edge(robot.shape, gate, distance);
                edges.push_back(edge);
                // push in the adj map the robot shape and the gate
                AdjacentNode adj_node = AdjacentNode(gate, distance);
                adj[robot.shape].push_back(adj_node);
                adj_node = AdjacentNode(robot.shape, distance);
                adj[gate].push_back(adj_node);
            }
        }

        // Compute the edges between robots and obstacles
        for(Robot robot : robots) {
            for(Polygon obstacle : obstacles) {
                for(Point point : obstacle.points) {
//                    std::cout << "Robot: " << robot.shape.x << " " << robot.shape.y << std::endl;
//                    std::cout << "Point: " << point.x << " " << point.y << std::endl;
                    if(!intersectsObstacle(robot.shape, point, obstacles)) {
                        double distance = sqrt(pow(robot.shape.x - point.x, 2) + pow(robot.shape.y - point.y, 2));
                        Edge edge = Edge(robot.shape, point, distance);
                        edges.push_back(edge);
                        // push in the adj map the robot shape and the point
                        AdjacentNode adj_node = AdjacentNode(point, distance);
                        adj[robot.shape].push_back(adj_node);
                        adj_node = AdjacentNode(robot.shape, distance);
                        adj[point].push_back(adj_node);
                    }
                }
            }
        }

        // Compute the edges of each obstacle
        for(Polygon obstacle: obstacles) {
            for (std::vector<Point> points : obstacle.getPolygonEdges()) {
                Point point = points[0];
                Point otherPoint = points[1];
                double distance = sqrt(pow(point.x - otherPoint.x, 2) + pow(point.y - otherPoint.y, 2));
                Edge edge = Edge(point, otherPoint, distance);
                edges.push_back(edge);
                // push in the adj map the point and the other point
                AdjacentNode adj_node = AdjacentNode(otherPoint, distance);
                adj[point].push_back(adj_node);
                adj_node = AdjacentNode(point, distance);
                adj[otherPoint].push_back(adj_node);
            }
        }

        // Compute the edges between obstacles
        for(int i = 0; i<obstacles.size(); i++) {
            for (int j = 0; j<obstacles.size(); j++) {
                if(i<j) {
                    Polygon obstacle = obstacles[i];
                    Polygon otherObstacle = obstacles[j];
                    for(Point point : obstacle.points) {
                        for(Point otherPoint : otherObstacle.points) {
                            if(!intersectsObstacle(point, otherPoint, obstacles)) {
                                double distance = sqrt(pow(point.x - otherPoint.x, 2) + pow(point.y - otherPoint.y, 2));
                                Edge edge = Edge(point, otherPoint, distance);
                                edges.push_back(edge);
                                // push in the adj map the point and the other point
                                AdjacentNode adj_node = AdjacentNode(otherPoint, distance);
                                adj[point].push_back(adj_node);
                                adj_node = AdjacentNode(point, distance);
                                adj[otherPoint].push_back(adj_node);
                            }
                        }
                    }
                }
            }
        }

        // Compute the edges between gate and obstacles
        for (Polygon obstacle : obstacles) {
            for (Point point : obstacle.points) {
                if (!intersectsObstacle(point, gate, obstacles)) {
                    double distance = sqrt(pow(point.x - gate.x, 2) + pow(point.y - gate.y, 2));
                    Edge edge = Edge(point, gate, distance);
                    edges.push_back(edge);
                    // push in the adj map the point and the gate
                    AdjacentNode adj_node = AdjacentNode(gate, distance);
                    adj[point].push_back(adj_node);
                    adj_node = AdjacentNode(point, distance);
                    adj[gate].push_back(adj_node);
                }
            }
        }
    }

    void VGraph::addNode(Node node) {
        nodes.push_back(node);
    }

    void VGraph::addEdge(Edge edge) {
        edges.push_back(edge);
    }

    std::vector<Edge> VGraph::getEdges() {
        return edges;
    }

//    std::vector<Node> VGraph::GetNodes() {
//        return nodes;
//    }

    std::map<Point, std::vector<AdjacentNode>> VGraph::getAdj() {
        return adj;
    }

    bool VGraph::existEdge(Node start, Node end) {
        for (Edge edge : edges) {
            if (edge.start.position == start.position && edge.end.position == end.position) {
                return true;
            }
        }
        return false;
    }

    Edge VGraph::getEdge(Node start, Node end) {
        for (Edge edge : edges) {
            if (edge.start.position == start.position && edge.end.position == end.position) {
                return edge;
            }
        }
        return Edge(Node(Point(0,0)), Node(Point(0,0)));
    }

    bool VGraph::intersectsObstacle(Point start, Point end, std::vector<Polygon> obstacles) {
//        std::cout << "start: " << start.x << ", " << start.y << std::endl;
//        std::cout << "end: " << end.x << ", " << end.y << std::endl;
//        exit(0);
        for(Polygon obstacle : obstacles) {
            if(intersectsLine(start, end, obstacle)) {
                return true;
            }
        }
        return false;
    };


    bool VGraph::intersectsLine(Point p1, Point p2, Polygon obstacle) {
//        std::cout << "p1: " << p1.x << ", " << p1.y << std::endl;
//        std::cout << "p2: " << p2.x << ", " << p2.y << std::endl;
//        std::cout << "edges: " << obstacle.getPolygonEdges().size() << std::endl;
//        exit(0);
        for(std::vector<Point> edge: obstacle.getPolygonEdges()) {
            if (p1 == edge[0] || p1 == edge[1] || p2 == edge[0] || p2 == edge[1]) {
                continue;
            }
//            std::cout << "edge: " << edge[0].x << ", " << edge[0].y << std::endl;
//            std::cout << "edge: " << edge[1].x << ", " << edge[1].y << std::endl;
//            std::cout << "Result " << lineIntersectsSegment(p1, p2, edge[0], edge[1]) << std::endl;
//            exit(0);
            if (lineIntersectsSegment(p1, p2, edge[0], edge[1])) {
                return true;
            }
        }
        return false;
    }

    bool VGraph::onSegment(Point p, Point q, Point r)
    {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;

        return false;
    }

    int VGraph::orientation(Point p, Point q, Point r)
    {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        int val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0;  // collinear

        return (val > 0)? 1: 2; // clock or counterclock wise
    }

    std::vector<Point> VGraph::shortestPath(Point origin, Point destination) {
        std::map<Point, double> dist;
        std::map<Point, Point> prev;

        std::set<std::pair<double, Point>> setVerticesProcessed;

        setVerticesProcessed.insert(std::make_pair(0, origin));
        dist[origin] = 0;

        // Loop until all shortest distances are finalized (or until the destination is found)
        while(!setVerticesProcessed.empty()) {
            // First vertex in the set - the min distance vertex
            std::pair<double, Point> tmp = *(setVerticesProcessed.begin());
            setVerticesProcessed.erase(setVerticesProcessed.begin());

            Point u = tmp.second;
//            std::cout << "Point: " << u.x << ", " << u.y << std::endl;

            // If we reached the destination, we can stop
            if(u == destination) {
                break;
            }

            // Loop through all adjacent vertices of the point u
            for (AdjacentNode e : adj[u]) {
                Point v = e.point;
                // The weight is simply the distance between the points
                double weight = e.distance;

                // If there is a shortest path from v through u (or it's the first one we find)
                if (!dist.count(v) || dist[v] > dist[u]+weight) {
                    // If we already had a distance from v through u, erase it (then we will insert the new one)
                    if (dist.count(v)) {
                        setVerticesProcessed.erase(setVerticesProcessed.find(std::make_pair(dist[v], v)));
                        prev.erase(prev.find(v));
                    }

                    // Update the distance from the point v
                    dist[v] = dist[u] + weight;
//                    std::cout << "Distance: " << dist[v] << std::endl;

                    setVerticesProcessed.insert(std::make_pair(dist[v], v));
                    prev.insert(std::make_pair(v,u));
                }
            }
        }

        // Reconstruct the path using prev. Start from destination
        std::vector<Point> path;
        while(true){
            path.push_back(destination);
            if(destination == origin) break;
            destination = prev.find(destination)->second;
        }
        // Reverse the path
        reverse(path.begin(), path.end());

        return path;
    }

    // The main function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    bool VGraph::lineIntersectsSegment(Point p1, Point q1, Point p2, Point q2)
    {
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
