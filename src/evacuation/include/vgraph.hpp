//
// Created by luca on 18/08/23.
//

#ifndef VGRAPH_HPP
#define VGRAPH_HPP

#include <vector>
#include <map>
#include "environment.hpp"

namespace evacuation::vgraph {
    class VGraph {
    public:
        VGraph(std::vector<Robot> robots, std::vector<Polygon> obstacles, Pose gate);

        static bool intersectsObstacles(Point start, Point end, const std::vector<Polygon> &obstacles);

        static bool intersectsObstacle(Point p1, Point p2, Polygon obstacle);

        static bool lineIntersectsSegment(Point p1, Point q1, Point p2, Point q2);

        static int direction(Point p, Point q, Point r);

        static bool areCollinearAndOverlapping(Point a1, Point b1, Point a2);

        std::vector<Point> shortestPath(Point origin, Point destination);

    private:
        struct AdjacentPoint {
            Point point;
            double distance;

            AdjacentPoint(Point point, double distance) : point(point), distance(distance) {}
        };

        struct Edge {
            Point start;
            Point end;
            double weight;

            Edge(Point start, Point end, double weight = 0) : start(start), end(end), weight(weight) {}
        };

        std::vector<Point> nodes;
        std::vector<Edge> edges;
        std::vector<Robot> robots;
        std::vector<Polygon> obstacles;
        Polygon map;
        std::map<Point, std::vector<AdjacentPoint>> adj;
    };
}

#endif