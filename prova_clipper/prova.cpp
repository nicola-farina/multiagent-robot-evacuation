#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "clipper/clipper.hpp"
#include <iostream>
#include <vector>

// Define the Point structure
struct Point {
    double x;
    double y;
    double z;
};

// Define the Polygon struct with points as a list of Point
struct Polygon {
    std::vector<Point> points;
};

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

int main() {
    ClipperLib::Path srcPoly;
    ClipperLib::Paths newPoly;

    Polygon polygon;

    // Add some points to the polygon
    Point p1 = {0.0, 0.0, 0.0};
    Point p2 = {0.0, 1.0, 0.0};
    Point p3 = {1.0, 1.0, 0.0};
    Point p4 = {1.0, 0.0, 0.0};

    polygon.points.push_back(p1);
    polygon.points.push_back(p2);
    polygon.points.push_back(p3);
    polygon.points.push_back(p4);

    std::vector<Point> result = enlarge(polygon.points, 1);
    for (int i = 0; i < result.size(); i++)
    {
        std::cout << "x: " << result[i].x << " y: " << result[i].y << std::endl;
    }

//    for (const Point &p : polygon.points) {
//        // Directly use integer coordinates without scaling
//        srcPoly << ClipperLib::IntPoint(p.x, p.y);
//    }
//
//    ClipperLib::ClipperOffset co;
//    if (polygon.points.size() == 3) {
//        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedLine);
//    } else {
//        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
//    }
//
//    const double OFFSET = 2.0; // Set the desired offset value here
//
//    co.Execute(newPoly, OFFSET * 1000); // Scale the offset
//
//    for (const ClipperLib::Path &path : newPoly) {
//        for (const ClipperLib::IntPoint &pt : path) {
//            double x = pt.X / 1000.0; // Scale back to original coordinates
//            double y = pt.Y / 1000.0;
//            std::cout << "x: " << x << " y: " << y << std::endl;
//        }
//    }

    return 0;
}
