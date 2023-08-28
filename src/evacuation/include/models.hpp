//
// Created by luca on 14/08/23.
//

#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <utility>
#include <vector>
#include <iostream>
#include "clipper.hpp"

struct Point {
    double x, y;

    explicit Point(double x = -1, double y = -1) : x(x), y(y) {}

    bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point &point2) const {
        return (x < point2.x) || (x == point2.x && y < point2.y);
    }
};

struct Pose {
    double x;
    double y;
    double th;

    Pose() = default;

    Pose(double x, double y, double th) : x(x), y(y), th(th) {}

    Point getPosition() {
        return Point(x, y);
    }
};

struct Polygon {
    std::vector<Point> points;

    explicit Polygon(std::vector<Point> points = std::vector<Point>()) : points(std::move(points)) {}

    std::vector<std::vector<Point>> getPolygonEdges() {
        std::vector<std::vector<Point>> edges;
        for (std::vector<Point>::size_type i = 0; i < points.size(); i++) {
            std::vector<Point> edge;
            edge.push_back(points[i]);
            edge.push_back(points[(i + 1) % points.size()]);
            edges.push_back(edge);
        }
        return edges;
    }
};

struct Robot {
    Pose pose;
    int index;
    std::string name;

    Robot() = default;

    explicit Robot(Pose initialPose, int index, const std::string &name = "shelfino") :
            pose(initialPose),
            index(index),
            name(name + "/" + std::to_string(index)) {}

    Point getPosition() {
        return pose.getPosition();
    }

    double getOrientation() {
        return pose.th;
    }
};

namespace environment {

    /*class Robot {
    public:
        Robot(Pose initialPose, int id, const std::string &name) : initialPose(initialPose), id(id), name(name + "/" + std::to_string(id)) {}

    private:
        Pose initialPose;
        int id;
        std::string name;

    };*/

    /**
     * @brief Class used to create the visibility graph given a certain environment
     */
    class Environment {
    public:
        Environment(
                Polygon map,
                const std::vector<Polygon> &obstacles,
                const std::vector<Robot> &robots,
                const Pose &gate
        ) : map(std::move(map)), robots(robots), gate(gate) {
            for (const Polygon &obstacle: obstacles) {
                if (this->isInside(obstacle)) {
                    std::cout << "Obstacle is entirely inside the map." << std::endl;
                    this->obstacles.push_back(obstacle);
                } else {
                    Polygon reshapedObstacle = reshapeToMapBoundary(obstacle);
                    if (!reshapedObstacle.points.empty()) {
                        std::cout << "Obstacle reshaped to touch map boundary:" << std::endl;
                        this->obstacles.push_back(reshapedObstacle);
                    } else {
                        std::cout << "Obstacle does not intersect with the map." << std::endl;
                    }
                }
            }
        }

        // Getter methods
        Polygon getMap();

        std::vector<Polygon> getObstacles();

        Polygon getObstacle(int i);

        std::vector<Robot> getRobots();

        Robot getRobot(int i);

        Pose getGate();

    private:
        Polygon map;
        std::vector<Polygon> obstacles;
        std::vector<Robot> robots;
        Pose gate;

        bool isInside(Polygon obstacle);

        Polygon reshapeToMapBoundary(Polygon obstacle);
    };
}

#endif //ENVIRONMENT_HPP