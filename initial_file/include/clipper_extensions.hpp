//
// Created by luca on 14/08/23.
//

#ifndef INITIAL_FILE_CLIPPER_EXTENSIONS_H
#define INITIAL_FILE_CLIPPER_EXTENSIONS_H

#include <clipper.hpp>

void printSolution(std::vector<ClipperLib::IntPoint> startingPoints, ClipperLib::Paths solution);

std::vector<Point> enlarge(std::vector<Point> points, double offset);

std::vector<std::vector<Point>> enlargeObstaclesWithTwoOffsets(std::vector<Point> polygon, double offset);

std::vector<std::vector<std::vector<Point>>> enlargeAndJoinObstacles(std::vector<Point> polygonsList, double offset);

#endif //INITIAL_FILE_CLIPPER_EXTENSIONS_H
