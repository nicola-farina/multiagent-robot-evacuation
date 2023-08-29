//
// Created by luca on 14/08/23.
//

#ifndef CLIPPER_EXTENSIONS_H
#define CLIPPER_EXTENSIONS_H

#include "clipper.hpp"
#include "environment.hpp"
#include <vector>

std::vector<evacuation::Point> enlarge(std::vector<evacuation::Point> points, double offset);

std::vector<std::vector<evacuation::Polygon>> enlargeAndJoinObstacles(std::vector<evacuation::Polygon> polygonsList, double offset);

#endif