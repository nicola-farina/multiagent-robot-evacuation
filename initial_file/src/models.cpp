#include "models.hpp"

using namespace environment;

// Getter methods
Polygon Environment::getMap() {
    return map;
}

std::vector<Polygon> Environment::getObstacles() {
    return obstacles;
}

Polygon Environment::getObstacle(int i) {
    return obstacles[i];
}

std::vector<Robot> Environment::getRobots() {
    return robots;
}

Robot Environment::getRobot(int i) {
    return robots[i];
}

std::vector<Polygon> Environment::getGates() {
    return gates;
}

Polygon Environment::reshapeToMapBoundary(Polygon obstacle) {
    // Initialize the Clipper library
    ClipperLib::Clipper clipper;

    // Transform the two polygons into Clipper Paths
    ClipperLib::Path mapPath;
    for (Point point : this->map.points) {
        mapPath << ClipperLib::IntPoint(point.x, point.y);
    }

    ClipperLib::Path obstaclePath;
    for (Point point : obstacle.points) {
        obstaclePath << ClipperLib::IntPoint(point.x, point.y);
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
        for (const ClipperLib::IntPoint &point : solution[0]) {
            solutionPolygon.points.emplace_back(point.X, point.Y);
        }
        return solutionPolygon;
    } else {
        // No intersection, return an empty polygon
        return Polygon(std::vector<Point>());
    }
}

bool Environment::isInside(Polygon obstacle) {
    // Initialize the Clipper library
    ClipperLib::Clipper clipper;

    // Transform the two polygons into Clipper Paths
    ClipperLib::Path mapPath;
    for (Point point : this->map.points) {
        mapPath << ClipperLib::IntPoint(point.x, point.y);
    }

    ClipperLib::Path obstaclePath;
    for (Point point : obstacle.points) {
        obstaclePath << ClipperLib::IntPoint(point.x, point.y);
    }

    for (const ClipperLib::IntPoint &vertex : obstaclePath) {
        if (!ClipperLib::PointInPolygon(vertex, mapPath)) {
            return false;
        }
    }
    return true;
}
