//
// Created by luca on 14/08/23.
//

#include <valarray>
#include "dubins_utils.hpp"

namespace dubins {

    double sinc(double t) {
        double s;
        if (std::abs(t) < 0.002) {
            s = 1 - pow(t, 2) / 6 * (1 - pow(t, 2) / 20);
        } else {
            s = sin(t) / t;
        }
        return s;
    }

    double mod2pi(double ang) {
        double out = ang;
        while (out < 0) {
            out = out + 2 * M_PI;
        }
        while (out >= 2 * M_PI) {
            out = out - 2 * M_PI;
        }
        return out;
    }

    double rangeSymm(double ang) {
        double out = ang;
        while (out <= -M_PI) {
            out = out + 2 * M_PI;
        }
        while (out > M_PI) {
            out = out - 2 * M_PI;
        }
        return out;
    }

    double crossProduct(DubinsPoint a, DubinsPoint b) {
        return a.x * b.y - a.y * b.x;
    }

    double dot2D(DubinsPoint a, DubinsPoint b) {
        return a.x * b.x + a.y * b.y;
    }

    int getOrientation(evacuation::Point p, evacuation::Point q, evacuation::Point r) {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        double val = (q.y - p.y) * (r.x - q.x) -
                     (q.x - p.x) * (r.y - q.y);

        if (val == 0)
            return 0; // collinear

        return (val < 0) ? 1 : -1; // clock or counterclock wise
    }
}