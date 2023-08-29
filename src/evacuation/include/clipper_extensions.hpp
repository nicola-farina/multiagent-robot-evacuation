//
// Created by luca on 14/08/23.
//

#ifndef CLIPPER_EXTENSIONS_HPP
#define CLIPPER_EXTENSIONS_HPP

#include "clipper.hpp"
#include "environment.hpp"
#include <vector>

namespace ClipperLibExtensions {
    std::vector<evacuation::Point> enlarge(std::vector<evacuation::Point> points, double offset);

    std::vector<evacuation::Polygon> enlargeObstaclesWithTwoOffsets(const evacuation::Polygon &polygon, double offset);

    std::vector<std::vector<evacuation::Polygon>> enlargeAndJoinObstacles(const std::vector<evacuation::Polygon> &polygonsList, double offset);
}

#endif