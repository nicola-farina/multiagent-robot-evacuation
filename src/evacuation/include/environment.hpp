//
// Created by luca on 14/08/23.
//

#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <utility>
#include <vector>
#include <iostream>
#include "clipper.hpp"
#include "robot.hpp"

namespace evacuation {

    struct Point {
        double x{}, y{};

        Point();

        Point(double x, double y);

        bool operator==(const Point& other) const;

        bool operator<(const Point& other) const;
    };

    struct Pose {
        Point position;
        double th;

        Pose();

        Pose(double x, double y, double th);
    };

    struct Polygon {
        std::vector<Point> points;

        Polygon();

        explicit Polygon(std::vector<Point> points);

        std::vector<std::vector<Point>> getPolygonEdges();
    };

    class Environment {
    public:
        Environment(Polygon map, const std::vector<Polygon> &obstacles, const std::vector<Robot> &robots, const Pose &gate);

        Polygon getMap();

        std::vector<Polygon> getObstacles();

    private:
        Polygon map;
        std::vector<Polygon> obstacles;
        std::vector<Robot> robots;
        Pose gate;

        bool isInside(const Polygon& obstacle);

        Polygon reshapeToMapBoundary(const Polygon& obstacle);
    };

}

#endif