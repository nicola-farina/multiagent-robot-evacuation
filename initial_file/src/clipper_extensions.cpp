#include "clipper.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>


/**
 * @brief Print a clipper polygon offsetting solution using OpenCV
 *
 * @param startingPoints Points of the original polygon
 * @param solution Solution given by clipper
 */
void printSolution(std::vector<ClipperLib::IntPoint> startingPoints, ClipperLib::Paths solution)
{
    // Create opencv plotting tool
    cv::Mat plot(1500, 1500, CV_8UC3, cv::Scalar(255, 255, 255));



    // Get the initial path
    ClipperLib::Path subj;
    for (int i = 0; i < startingPoints.size(); i++)
    {
        subj.push_back(startingPoints[i]);
    }
    // Draw the initial path
    cv::line(plot, cv::Point2f(subj.at(subj.size() - 1).X, subj.at(subj.size() - 1).Y), cv::Point2f(subj.at(0).X, subj.at(0).Y), cv::Scalar(255, 0, 0), 1);
    for (unsigned int j = 1; j < subj.size(); j++)
    {
        cv::line(plot, cv::Point2f(subj.at(j - 1).X, subj.at(j - 1).Y), cv::Point2f(subj.at(j).X, subj.at(j).Y), cv::Scalar(255, 0, 0), 1);
    }

    // Draw the solution
    for (unsigned int i = 0; i < solution.size(); i++)
    {
        ClipperLib::Path path = solution.at(i);
        cv::line(plot, cv::Point2f(path.at(path.size() - 1).X, path.at(path.size() - 1).Y), cv::Point2f(path.at(0).X, path.at(0).Y), cv::Scalar(255, 255, 0), 2);
        for (unsigned int j = 1; j < path.size(); j++)
        {
            std::cout << path.at(j - 1).X << " , " << path.at(j - 1).Y << " - " << path.at(j).X << " , " << path.at(j).Y << "\n";
            cv::line(plot, cv::Point2f(path.at(j - 1).X, path.at(j - 1).Y), cv::Point2f(path.at(j).X, path.at(j).Y), cv::Scalar(255, 255, 0), 2);
        }
    }
//    cv::flip(plot, plot, 0);
//    cv::namedWindow("Clipper", cv::WINDOW_NORMAL);
//    int desiredWidth = 1500;
//    int desiredHeight = 800;
//    cv::Mat resizedPlot;
//    cv::resize(plot, resizedPlot, cv::Size(desiredWidth, desiredHeight));
    cv::imshow("Clipper", plot);
    cv::waitKey(0);
}



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
    for (int i = 0; i < points.size(); i++)
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
    printSolution(subj, solution);

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
std::vector<std::vector<Point>> enlargeObstaclesWithTwoOffsets(std::vector<Point> polygon, double offset){
    double variant = 2.5;
    std::vector<Point> newPath;

    //Convert the polygon to the data structure for enlarge, we need a vector of visgraph points
    for(unsigned int i = 0; i < polygon.size(); i++){
        newPath.push_back(Point{polygon[i].x, polygon[i].y});
    }
    std::vector<Point> bigSolution;
    std::vector<Point> smallSolution;
    //We enlarge the polygon by using an offset and an offset + (offset/variant)
    smallSolution = enlarge(newPath, offset);
    bigSolution = enlarge(newPath, offset + (offset/variant));
    //Take both solutions, push them back a vector, return it
    std::vector<std::vector<Point>> finalResult;
    finalResult.push_back(bigSolution);
    finalResult.push_back(smallSolution);

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
std::vector<std::vector<std::vector<Point>>> enlargeAndJoinObstacles(std::vector<Point> polygonsList, double offset){

    std::vector<std::vector<Point>> bigPolygons;
    std::vector<std::vector<Point>> smallPolygons;

    //We convert each polygon to a "slightly bigger" and a "bigger" version and then we push then into different vectors
    for (int i = 0; i < polygonsList.size(); i++){
        std::vector<std::vector<Point>> results;
        results = enlargeObstaclesWithTwoOffsets(polygonsList[i], offset);
        bigPolygons.push_back(results[0]);
        smallPolygons.push_back(results[1]);
        results.clear();
    }

    //We join all the "bigPolygons"
    ClipperLib::Paths subj(bigPolygons.size()), solution;

    for (unsigned int i = 0; i < bigPolygons.size(); i++){
        for (unsigned int j = 0; j < bigPolygons[i].size(); j++) {
            subj[i].push_back(ClipperLib::IntPoint(bigPolygons[i][j].x*1000, bigPolygons[i][j].y*1000));
        }
    }
    ClipperLib::Clipper c;
    c.AddPaths(subj, ClipperLib::ptSubject, true);
    c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero);

    CleanPolygons(solution);

    //We join also all the "smaller" polygons
    ClipperLib::Paths subj1(smallPolygons.size()), solution1;

    for (unsigned int i = 0; i < smallPolygons.size(); i++){
        for (unsigned int j = 0; j < smallPolygons[i].size(); j++) {
            subj1[i].push_back(ClipperLib::IntPoint(smallPolygons[i][j].x*1000, smallPolygons[i][j].y*1000));
        }
    }

    ClipperLib::Clipper c1;
    c1.AddPaths(subj1, ClipperLib::ptSubject, true);
    c1.Execute(ClipperLib::ctUnion, solution1, ClipperLib::pftNonZero);

    CleanPolygons(solution1);

    //We return all the enlarged and joined polygons
    std::vector<std::vector<std::vector<Point>>> returnValues;

    std::vector<std::vector<Point>> intermediateValues;

    for (unsigned int i = 0; i < solution.size(); i++){
        ClipperLib::Path path = solution.at(i);
        std::vector<Point> newPath;
        if (Orientation(path)) {
            for(ClipperLib::IntPoint p : path){
                newPath.push_back(Point{p.X/1000.0, p.Y/1000.0});
            }
        }
        if (!newPath.empty())
            intermediateValues.push_back(newPath);
        newPath.clear();
    }

    returnValues.push_back(intermediateValues);

    intermediateValues.clear();

    for (unsigned int i = 0; i < solution1.size(); i++){
        ClipperLib::Path path = solution1.at(i);
        std::vector<Point> newPath;
        if (Orientation(path)) {
            for(ClipperLib::IntPoint p : path){
                newPath.push_back(Point{p.X/1000.0, p.Y/1000.0});
            }
        }
        if (!newPath.empty())
            intermediateValues.push_back(newPath);
        newPath.clear();
    }

    returnValues.push_back(intermediateValues);

    return returnValues;
}
