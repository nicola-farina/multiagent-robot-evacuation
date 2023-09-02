//
// Created by luca on 14/08/23.
//

#ifndef DUBINS_UTILS_HPP
#define DUBINS_UTILS_HPP

#include "environment.hpp"

namespace dubins {

    struct DubinsPoint {
        double x, y, th;

        DubinsPoint(double x = -1, double y = -1, double th = -1) : x(x), y(y), th(th) {}
    };

    double sinc(double t);

    double mod2pi(double angle);

    double rangeSymm(double angle);

    double crossProduct(DubinsPoint a, DubinsPoint b);

    double dot2D(DubinsPoint a, DubinsPoint b);

    int getOrientation(evacuation::Point p, evacuation::Point q, evacuation::Point r);

}

#endif