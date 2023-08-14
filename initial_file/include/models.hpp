//
// Created by luca on 14/08/23.
//

#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <vector>
#include <iostream>
#include "clipper.hpp"

struct Point {
    double x, y, z;
    Point(double x = -1, double y = -1, double z = -1) : x(x), y(y), z(z) {}
};

struct Polygon {
    std::vector<Point> points;

    Polygon(std::vector<Point> points = std::vector<Point>()) : points(points) {}
};

struct Robot {
    Polygon shape;
    double radius;

    Robot(Polygon shape, double radius) : shape(shape), radius(radius) {}
};


namespace environment
{
    /**
     * @brief Class used to create the visibility graph given a certain environment
     *
     */

    class Environment {
    public:
        // Constructor
        Environment(const Polygon& map,
                    const std::vector<Polygon>& obstacles,
                    const std::vector<Robot>& robots,
                    const std::vector<Polygon>& gates)
                : map(map), robots(robots), gates(gates) {
            for (const Polygon& obstacle : obstacles) {
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

        std::vector<Polygon> getGates();

    private:
        Polygon map;
        std::vector<Polygon> obstacles;
        std::vector<Robot> robots;
        std::vector<Polygon> gates;

        bool isInside(Polygon obstacle);

        Polygon reshapeToMapBoundary(Polygon obstacle);
    };
}

#endif //ENVIRONMENT_HPP