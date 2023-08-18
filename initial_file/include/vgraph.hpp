//
// Created by luca on 18/08/23.
//

#ifndef INITIAL_FILE_VGRPAH_H
#define INITIAL_FILE_VGRPAH_H

#include <vector>
#include "models.hpp"

namespace vgraph {

    struct Node {
        Point position;

        Node(Point position) : position(position) {}
    };

    struct Edge {
        Node start;
        Node end;
        double weight;

        Edge(Node start, Node end, double weight = 0) : start(start), end(end), weight(weight) {}
    };

    class VGraph {
    public:
        VGraph(std::vector<Robot> robots, std::vector<Polygon> obstacles, Point gate);

        void addNode(Node node);

        void addEdge(Edge edge);

        std::vector<Edge> getEdges();

        bool existEdge(Node start, Node end);

        Edge getEdge(Node start, Node end);

        bool intersectsObstacle(Point start, Point end, std::vector<Polygon> obstacles);

        bool intersectsLine(Point p1, Point p2, Polygon obstacle);

        bool lineIntersectsSegment(Point p1, Point q1, Point p2, Point q2);

        bool onSegment(Point p, Point q, Point r);

        int orientation(Point p, Point q, Point r);

    private:
        std::vector<Node> nodes;
        std::vector<Edge> edges;
        std::vector<Robot> robots;
        std::vector<Polygon> obstacles;
        Point gate;
        Polygon map;
    };
}


#endif //INITIAL_FILE_VGRPAH_H
