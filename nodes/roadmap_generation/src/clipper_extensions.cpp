#include "clipper_extensions.hpp"
#include "clipper.hpp"
#include "models.hpp"
#include <iostream>
#include <vector>

/*
* @brief Clipper Polygon Offsetting helper function
* We multiply the points by 1000 because Clipper works best under the assumption of working on a certain scale
*
* @param points Points of the original polygon
* @param offset Offset that must be used to enlarge or shrink the polygon
* @return std::vector<visgraph::Point> Array of points of the resulting polygon
*/
std::vector<Point> enlarge(std::vector<Point> points, double offset)
{
    ClipperLib::Path subj;
    ClipperLib::Paths solution;
    for (std::vector<Point>::size_type i = 0; i < points.size(); i++)
    {
        subj << ClipperLib::IntPoint(points[i].x*1000, points[i].y*1000);
    }

    ClipperLib::ClipperOffset co;
    //Jtmiter "cuts" or "smooths" certain angles that can be generated
    co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    co.Execute(solution, offset*1000.0);

    CleanPolygons(solution);

    //Conversion from IntPoint back to visgraph::Point
    std::vector<Point> result;
    if (solution.size() > 0) {
        for (ClipperLib::IntPoint p : solution[0]) {
            result.push_back(Point{p.X / 1000.0, p.Y / 1000.0});
        }
    }
//    printSolution(subj, solution);

    // printSolution(subj, solution);

    return result;
}


/**
 * @brief Given some obstacles and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 *
 * @param polygon Obstacles we are considering
 * @param offset Offset for obstacle offsetting
 * @return std::vector<std::vector<student::Point>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones
 */
std::vector<Polygon> enlargeObstaclesWithTwoOffsets(Polygon polygon, double offset){
    double variant = 2.5;
    std::vector<Point> newPath;

    std::vector<Point> polygonPoints = polygon.points;
    //Convert the polygon to the data structure for enlarge, we need a vector of visgraph points
    for(unsigned int i = 0; i < polygonPoints.size(); i++){
        newPath.push_back(Point{polygonPoints[i].x, polygonPoints[i].y});
    }
    Polygon bigSolution;
    Polygon smallSolution;
    //We enlarge the polygon by using an offset and an offset + (offset/variant)
    smallSolution = enlarge(newPath, offset);
    bigSolution = enlarge(newPath, offset + (offset/variant));
    //Take both solutions, push them back a vector, return it
    std::vector<Polygon> finalResult;
    Polygon smallPolygon, bigPolygon;
    for(unsigned int i = 0; i < smallSolution.points.size(); i++){
        smallPolygon.points.push_back(Point{smallSolution.points[i].x, smallSolution.points[i].y});
    }
    for(unsigned int i = 0; i < bigSolution.points.size(); i++){
        bigPolygon.points.push_back(Point{bigSolution.points[i].x, bigSolution.points[i].y});
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
 * @return std::vector<std::vector<std::vector<visgraph::Point>>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones, joined in case of collisions
 */
std::vector<std::vector<Polygon>> enlargeAndJoinObstacles(std::vector<Polygon> polygonsList, double offset){

    std::vector<Polygon> bigPolygons;
    std::vector<Polygon> smallPolygons;

    //We convert each polygon to a "slightly bigger" and a "bigger" version and then we push then into different vectors
    for (std::vector<Polygon>::size_type i = 0; i < polygonsList.size(); i++){
        std::vector<Polygon> results;
        results = enlargeObstaclesWithTwoOffsets(polygonsList[i], offset);
        bigPolygons.push_back(results[0]);
        smallPolygons.push_back(results[1]);
        results.clear();
    }

    //We join all the "bigPolygons"
    ClipperLib::Paths subj(bigPolygons.size()), solution;

    for (unsigned int i = 0; i < bigPolygons.size(); i++){
        for (unsigned int j = 0; j < bigPolygons[i].points.size(); j++) {
            subj[i].push_back(ClipperLib::IntPoint(bigPolygons[i].points[j].x*1000, bigPolygons[i].points[j].y*1000));
        }
    }
    ClipperLib::Clipper c;
    c.AddPaths(subj, ClipperLib::ptSubject, true);
    c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero);

    CleanPolygons(solution);

    //We join also all the "smaller" polygons
    ClipperLib::Paths subj1(smallPolygons.size()), solution1;

    for (unsigned int i = 0; i < smallPolygons.size(); i++){
        for (unsigned int j = 0; j < smallPolygons[i].points.size(); j++) {
            subj1[i].push_back(ClipperLib::IntPoint(smallPolygons[i].points[j].x*1000, smallPolygons[i].points[j].y*1000));
        }
    }

    ClipperLib::Clipper c1;
    c1.AddPaths(subj1, ClipperLib::ptSubject, true);
    c1.Execute(ClipperLib::ctUnion, solution1, ClipperLib::pftNonZero);

    CleanPolygons(solution1);

    //We return all the enlarged and joined polygons
    std::vector<std::vector<Polygon>> returnValues;

    std::vector<Polygon> intermediateValues;

    for (unsigned int i = 0; i < solution.size(); i++){
        ClipperLib::Path path = solution.at(i);
        Polygon newPath;
        if (Orientation(path)) {
            for(ClipperLib::IntPoint p : path){
                newPath.points.push_back(Point{p.X/1000.0, p.Y/1000.0});
            }
        }
        if (!newPath.points.empty())
            intermediateValues.push_back(newPath);
        newPath.points.clear();
    }

    returnValues.push_back(intermediateValues);

    intermediateValues.clear();

    for (unsigned int i = 0; i < solution1.size(); i++){
        ClipperLib::Path path = solution1.at(i);
        Polygon newPath;
        if (Orientation(path)) {
            for(ClipperLib::IntPoint p : path){
                newPath.points.push_back(Point{p.X/1000.0, p.Y/1000.0});
            }
        }
        if (!newPath.points.empty())
            intermediateValues.push_back(newPath);
        newPath.points.clear();
    }

    returnValues.push_back(intermediateValues);

    return returnValues;
}
