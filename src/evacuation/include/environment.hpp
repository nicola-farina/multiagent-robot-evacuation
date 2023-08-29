//
// Created by luca on 14/08/23.
//

#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <vector>
#include <string>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/time.hpp>

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

        geometry_msgs::msg::PoseStamped toPoseStamped(rclcpp::Time time, std::string frameId) const;
    };

    struct Polygon {
        std::vector<Point> points;

        Polygon();

        explicit Polygon(std::vector<Point> points);

        std::vector<std::vector<Point>> getPolygonEdges();
    };

    struct Robot {
        Pose pose;
        int id;

        Robot();

        explicit Robot(int id);

        Point getPosition();

        double getOrientation();

        std::string getName();
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