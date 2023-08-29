/**
 * @file utils.cpp
 * @brief Functions used to calculate the Dubins Curves
 * @version 0.1
 * @date 2022-01-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "utils.hpp"

namespace dubins {

    /**
     * @brief Implementation of function sinc(t)
     *
     * @param t TODO
     * @return double 1 for t==0, and sin(t)/t otherwise
     */
    double sinc(double t) {
        double s;
        if (std::abs(t) < 0.002) {
            s = 1 - pow(t, 2) / 6 * (1 - pow(t, 2) / 20);
        } else {
            s = sin(t) / t;
        }
        return s;
    }

    /**
     * @brief Normalize an angle (in range [0,2*pi))
     *
     * @param ang Angle to normalize
     * @return double Normalized angle
     */
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

    /**
     * @brief Normalize an angular difference (range (-pi, pi])
     *
     * @param ang Angular difference to normalize
     * @return double Normalized angular difference
     */
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

    /**
     * @brief Calculates the cross product between two Points
     *
     * @param a First input point
     * @param b Second input point
     * @return double Cross product between provided points
     */
    double crossProduct(DubinsPoint a, DubinsPoint b) {
        return a.x * b.y - a.y * b.x;
    }

    /**
     * @brief Calculates the dot product between two Points
     *
     * @param a First input point
     * @param b Second input point
     * @return double Dot product between provided points
     */
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