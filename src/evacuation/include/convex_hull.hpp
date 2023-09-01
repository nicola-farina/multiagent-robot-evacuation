//
// Created by nicola on 31/08/23.
//

#ifndef EVACUATION_CONVEX_HULL_HPP
#define EVACUATION_CONVEX_HULL_HPP

#include "environment.hpp"

namespace convex {
    std::vector<evacuation::Polygon> getConvexHull(std::vector<evacuation::Polygon> poly);

    std::vector<evacuation::Point> orderPoints(std::vector<evacuation::Point> points);
}

#endif
