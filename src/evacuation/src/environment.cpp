//
// Created by luca on 14/08/23.
//

#include "environment.hpp"

#include <utility>
#include <tf2/LinearMath/Quaternion.h>
#include "clipper.hpp"
#include "angle_utils.hpp"

namespace evacuation {
    Point::Point() = default;

    Point::Point(double x, double y) : x(x), y(y) {}

    bool Point::operator==(const evacuation::Point &other) const {
        return x == other.x && y == other.y;
    }

    bool Point::operator<(const evacuation::Point &other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }

    Pose::Pose() = default;

    Pose::Pose(double x, double y, double th) : position(Point(x, y)), th(th) {}

    geometry_msgs::msg::PoseStamped Pose::toPoseStamped(rclcpp::Time time, std::string frameId) const {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header.stamp = time;
        stamped.header.frame_id = std::move(frameId);
        stamped.pose.position.x = position.x;
        stamped.pose.position.y = position.y;
        stamped.pose.position.z = 0.0;
        stamped.pose.orientation = angle::yawToQuaternionMsg(th);
        return stamped;
    }

    Polygon::Polygon() = default;

    Polygon::Polygon(std::vector<Point> points) : points(std::move(points)) {}

    std::vector<std::vector<Point>> Polygon::getPolygonEdges() {
        std::vector<std::vector<Point>> edges;
        for (std::vector<Point>::size_type i = 0; i < points.size(); i++) {
            std::vector<Point> edge;
            edge.push_back(points[i]);
            edge.push_back(points[(i + 1) % points.size()]);
            edges.push_back(edge);
        }
        return edges;
    }

    Robot::Robot() = default;

    Robot::Robot(int id) : id(id) {}

    Point Robot::getPosition() {
        return pose.position;
    }

    double Robot::getOrientation() {
        return pose.th;
    }

    std::string Robot::getName() {
        return "shelfino" + std::to_string(id);
    }

    Environment::Environment(evacuation::Polygon map, const std::vector<Polygon> &obstacles, const std::vector<Robot> &robots, const evacuation::Pose &gate)
            : map(std::move(map))
            , robots(robots)
            , gate(gate) {
        for (const Polygon &obstacle: obstacles) {
            if (this->isInside(obstacle)) {
                // Obstacle is entirely inside the map
                this->obstacles.push_back(obstacle);
            } else {
                Polygon reshapedObstacle = reshapeToMapBoundary(obstacle);
                if (!reshapedObstacle.points.empty()) {
                    // Obstacle reshaped to touch map boundary
                    this->obstacles.push_back(reshapedObstacle);
                } else {
                    // Obstacle does not intersect with the map, do nothing
                }
            }
        }
    }

    Polygon Environment::getMap() {
        return map;
    }

    std::vector<Polygon> Environment::getObstacles() {
        return obstacles;
    }

    Polygon Environment::reshapeToMapBoundary(const Polygon &obstacle) {
        // Initialize the Clipper library
        ClipperLib::Clipper clipper;

        // Transform the two polygons into Clipper Paths
        ClipperLib::Path mapPath;
        for (Point point: this->map.points) {
            mapPath << ClipperLib::IntPoint((long) point.x * 1000, (long) point.y * 1000);
        }

        ClipperLib::Path obstaclePath;
        for (Point point: obstacle.points) {
            obstaclePath << ClipperLib::IntPoint((long) point.x * 1000, (long) point.y * 1000);
        }

        // Add the map and polygon to the Clipper subject
        clipper.AddPath(mapPath, ClipperLib::ptSubject, true);
        clipper.AddPath(obstaclePath, ClipperLib::ptClip, true);

        // Perform the intersection operation
        ClipperLib::Paths solution;
        clipper.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

        if (!solution.empty()) {
            // The first solution polygon represents the portion of the polygon inside the map
            // Translate the Clipper Path into a Polygon
            Polygon solutionPolygon;
            for (const ClipperLib::IntPoint &point: solution[0]) {
                solutionPolygon.points.emplace_back((double) point.X / 1000.0, (double) point.Y / 1000.0);
            }
            return solutionPolygon;
        } else {
            // No intersection, return an empty polygon
            return Polygon(std::vector<Point>());
        }
    }

    bool Environment::isInside(const Polygon &obstacle) {
        // Initialize the Clipper library
        ClipperLib::Clipper clipper;

        // Transform the two polygons into Clipper Paths
        ClipperLib::Path mapPath;
        for (Point point: this->map.points) {
            mapPath << ClipperLib::IntPoint((long) point.x * 1000, (long) point.y * 1000);
        }

        ClipperLib::Path obstaclePath;
        for (Point point: obstacle.points) {
            obstaclePath << ClipperLib::IntPoint((long) point.x * 1000, (long) point.y * 1000);
        }

        return std::all_of(
                obstaclePath.begin(), obstaclePath.end(),
                [&mapPath](ClipperLib::IntPoint vertex) { return ClipperLib::PointInPolygon(vertex, mapPath); }
        );
    }
}
