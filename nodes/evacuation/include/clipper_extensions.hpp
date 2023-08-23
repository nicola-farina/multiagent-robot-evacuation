//
// Created by luca on 14/08/23.
//

#ifndef INITIAL_FILE_CLIPPER_EXTENSIONS_H
#define INITIAL_FILE_CLIPPER_EXTENSIONS_H

#include "clipper.hpp"
#include "models.hpp"
#include <vector>

std::vector<Point> enlarge(std::vector<Point> points, double offset);

std::vector<std::vector<Polygon>> enlargeAndJoinObstacles(std::vector<Polygon> polygonsList, double offset);

std::vector<std::vector<Polygon>> enlargeAndJoinObstacles(std::vector<Polygon> polygonsList, double offset);

#endif //INITIAL_FILE_CLIPPER_EXTENSIONS_H
