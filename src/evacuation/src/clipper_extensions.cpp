#include "clipper_extensions.hpp"
#include "clipper.hpp"
#include "environment.hpp"
#include <vector>

/*
* @brief Clipper Polygon Offsetting helper function
* We multiply the points by 1000 because Clipper works best under the assumption of working on a certain scale
*
* @param points Points of the original polygon
* @param offset Offset that must be used to enlarge or shrink the polygon
* @return vector<visgraph::Point> Array of points of the resulting polygon
*/
using evacuation::Point;
using evacuation::Polygon;
using std::vector;

namespace ClipperLibExtensions {

    vector <Point> enlarge(vector <Point> points, double offset) {
        ClipperLib::Path subj;
        ClipperLib::Paths solution;
        for (vector<Point>::size_type i = 0; i < points.size(); i++) {
            subj << ClipperLib::IntPoint(points[i].x * 1000, points[i].y * 1000);
        }

        ClipperLib::ClipperOffset co;
        //Jtmiter "cuts" or "smooths" certain angles that can be generated
        co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        co.Execute(solution, offset * 1000.0);

        CleanPolygons(solution);

        //Conversion from IntPoint back to visgraph::Point
        vector<Point> result;
        if (!solution.empty()) {
            for (ClipperLib::IntPoint p: solution[0]) {
                result.emplace_back(p.X / 1000.0, p.Y / 1000.0);
            }
        }

        return result;
    }

/**
 * @brief Given some obstacles and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 *
 * @param polygon Obstacles we are considering
 * @param offset Offset for obstacle offsetting
 * @return vector<vector<student::Point>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones
 */
    vector <Polygon> enlargeObstaclesWithTwoOffsets(const Polygon &polygon, double offset) {
        double variant = 2.5;
        vector<Point> newPath;

        vector<Point> polygonPoints = polygon.points;
        //Convert the polygon to the data structure for enlarge, we need a vector of visgraph points
        for (auto &polygonPoint: polygonPoints) {
            newPath.emplace_back(polygonPoint.x, polygonPoint.y);
        }
        Polygon bigSolution;
        Polygon smallSolution;
        //We enlarge the polygon by using an offset and an offset + (offset/variant)
        smallSolution = Polygon(enlarge(newPath, offset));
        bigSolution = Polygon(enlarge(newPath, offset + (offset / variant)));
        //Take both solutions, push them back a vector, return it
        vector<Polygon> finalResult;
        Polygon smallPolygon, bigPolygon;
        for (auto &point: smallSolution.points) {
            smallPolygon.points.emplace_back(point.x, point.y);
        }
        for (auto &point: bigSolution.points) {
            bigPolygon.points.emplace_back(point.x, point.y);
        }
        finalResult.push_back(bigPolygon);
        finalResult.push_back(smallPolygon);

        return finalResult;
    }


/**
 * @brief Enlarge the obstacles using the function enlargeObstaclesWithTwoOffsets, then join them in case they collide between each other
 * The function also removes possible holes that can be formed during the join phase
 *
 * @param polygonsList All the obstacles
 * @param offset Offset for enlarging them
 * @return vector<vector<vector<visgraph::Point>>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones, joined in case of collisions
 */
    vector <vector<Polygon>> enlargeAndJoinObstacles(const vector <Polygon> &polygonsList, double offset) {

        vector<Polygon> bigPolygons;
        vector<Polygon> smallPolygons;

        //We convert each polygon to a "slightly bigger" and a "bigger" version and then we push then into different vectors
        for (const auto &i: polygonsList) {
            vector<Polygon> results;
            results = enlargeObstaclesWithTwoOffsets(i, offset);
            bigPolygons.push_back(results[0]);
            smallPolygons.push_back(results[1]);
            results.clear();
        }

        //We join all the "bigPolygons"
        ClipperLib::Paths subj(bigPolygons.size()), solution;

        for (unsigned int i = 0; i < bigPolygons.size(); i++) {
            for (unsigned int j = 0; j < bigPolygons[i].points.size(); j++) {
                subj[i].emplace_back(bigPolygons[i].points[j].x * 1000, bigPolygons[i].points[j].y * 1000);
            }
        }
        ClipperLib::Clipper c;
        c.AddPaths(subj, ClipperLib::ptSubject, true);
        c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero);

        CleanPolygons(solution);

        //We join also all the "smaller" polygons
        ClipperLib::Paths subj1(smallPolygons.size()), solution1;

        for (unsigned int i = 0; i < smallPolygons.size(); i++) {
            for (unsigned int j = 0; j < smallPolygons[i].points.size(); j++) {
                subj1[i].emplace_back(smallPolygons[i].points[j].x * 1000, smallPolygons[i].points[j].y * 1000);
            }
        }

        ClipperLib::Clipper c1;
        c1.AddPaths(subj1, ClipperLib::ptSubject, true);
        c1.Execute(ClipperLib::ctUnion, solution1, ClipperLib::pftNonZero);

        CleanPolygons(solution1);

        //We return all the enlarged and joined polygons
        vector<vector<Polygon>> returnValues;

        vector<Polygon> intermediateValues;

        for (const auto &path: solution) {
            Polygon newPath;
            if (Orientation(path)) {
                for (ClipperLib::IntPoint p: path) {
                    newPath.points.emplace_back(p.X / 1000.0, p.Y / 1000.0);
                }
            }
            if (!newPath.points.empty())
                intermediateValues.push_back(newPath);
            newPath.points.clear();
        }

        returnValues.push_back(intermediateValues);

        intermediateValues.clear();

        for (const auto &path: solution1) {
            Polygon newPath;
            if (Orientation(path)) {
                for (ClipperLib::IntPoint p: path) {
                    newPath.points.emplace_back(p.X / 1000.0, p.Y / 1000.0);
                }
            }
            if (!newPath.points.empty())
                intermediateValues.push_back(newPath);
            newPath.points.clear();
        }

        returnValues.push_back(intermediateValues);

        return returnValues;
    }
}
