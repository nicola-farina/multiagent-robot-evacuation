/**
 * @file dubins.cpp
 * @brief Implementation of the generation of paths composed by Dubins Curves
 * @version 0.1
 * @date 2022-01-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "dubins.hpp"
#include "utils.hpp"
#include "environment.hpp"

#include <iostream>
#include <cfloat>
#include <algorithm>
#include <vector>
#include <cmath>

/**
 * @brief Namespace for Dubins Curves generation
 *
 */
namespace dubins
{
    /**
     * @brief Construct a new Dubins:: Dubins object
     *
     * @param k_max Bound on maximum path curvature
     * @param discritizer_size Given a path of infinite points, what is the discritizer size? Expressed in meters
     */
    Dubins::Dubins(double k_max, double discritizer_size)
    {
        this->k_max = k_max;
        this->discritizer_size = discritizer_size;
    };

    /**
     * @brief Verify if the dubins solution curves are valid.
     *
     * @param curve_segments The curves to be verified
     * @param k0 TODO
     * @param k1 TODO
     * @param k2 TODO
     * @param th0 Starting angle
     * @param thf Final angle
     * @return true If the solution is valid
     * @return false If the solution is not valid
     */
    bool Dubins::areCurvesValid(CurveSegmentsResult *curve_segments, double k0, double k1, double k2, double th0, double thf)
    {
        double x0 = -1;
        double y0 = 0;
        double xf = 1;
        double yf = 0;

        double eq1 = x0 + curve_segments->s1 * sinc((1 / 2.) * k0 * curve_segments->s1) * cos(th0 + (1 / 2.) * k0 * curve_segments->s1) + curve_segments->s2 * sinc((1 / 2.) * k1 * curve_segments->s2) * cos(th0 + k0 * curve_segments->s1 + (1 / 2.) * k1 * curve_segments->s2) + curve_segments->s3 * sinc((1 / 2.) * k2 * curve_segments->s3) * cos(th0 + k0 * curve_segments->s1 + k1 * curve_segments->s2 + (1 / 2.) * k2 * curve_segments->s3) - xf;
        double eq2 = y0 + curve_segments->s1 * sinc((1 / 2.) * k0 * curve_segments->s1) * sin(th0 + (1 / 2.) * k0 * curve_segments->s1) + curve_segments->s2 * sinc((1 / 2.) * k1 * curve_segments->s2) * sin(th0 + k0 * curve_segments->s1 + (1 / 2.) * k1 * curve_segments->s2) + curve_segments->s3 * sinc((1 / 2.) * k2 * curve_segments->s3) * sin(th0 + k0 * curve_segments->s1 + k1 * curve_segments->s2 + (1 / 2.) * k2 * curve_segments->s3) - yf;
        double eq3 = rangeSymm(k0 * curve_segments->s1 + k1 * curve_segments->s2 + k2 * curve_segments->s3 + th0 - thf);

        bool Lpos = (curve_segments->s1 > 0) || (curve_segments->s2 > 0) || (curve_segments->s3 > 0);

        return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10 && Lpos);
    };

    /**
     * @brief Scale the input parameters to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
     *
     * @param x0 Starting x position
     * @param y0 Starting y position
     * @param th0 Starting angle
     * @param xf Final x position
     * @param yf Final y position
     * @param thf Final angle
     * @return DubinsResult* Scaled parameters
     */
    DubinsResult *Dubins::scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf)
    {
        double dx = xf - x0;
        double dy = yf - y0;
        double phi = atan2(dy, dx);
        double lambda = hypot(dx, dy) / 2;

        double sc_th0 = mod2pi(th0 - phi);
        double sc_thf = mod2pi(thf - phi);
        double sc_k_max = k_max * lambda;
        return new DubinsResult(sc_th0, sc_thf, sc_k_max, lambda);
    }

    /**
     * @brief Return to the initial scaling
     *
     * @param lambda TODO
     * @param curve_segments Current scaled parameters of our problem
     * @return CurveSegmentsResult*
     */
    CurveSegmentsResult *Dubins::scaleFromStandard(double lambda, CurveSegmentsResult *curve_segments)
    {
        return new CurveSegmentsResult(true, curve_segments->s1 * lambda, curve_segments->s2 * lambda, curve_segments->s3 * lambda);
    }

    CurveSegmentsResult *Dubins::curveLSL(double scaled_th0, double scaled_thf, double scaled_k_max)
    {
        double s1, s2, s3;
        double invK = 1.0 / scaled_k_max;
        double C = cos(scaled_thf) - cos(scaled_th0);
        double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);
        double temp1 = atan2(C, S);
        s1 = invK * mod2pi(temp1 - scaled_th0);
        double temp2 = 2 + 4 * pow(scaled_k_max, 2) - 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf));
        if (temp2 < 0)
        {
            s1 = 0;
            s2 = 0;
            s3 = 0;
            return new CurveSegmentsResult(false, s1, s2, s3);
        }
        s2 = invK * sqrt(temp2);
        s3 = invK * mod2pi(scaled_thf - temp1);
        return new CurveSegmentsResult(true, s1, s2, s3);
    }

    CurveSegmentsResult *Dubins::curveRSR(double scaled_th0, double scaled_thf, double scaled_k_max)
    {
        double s1, s2, s3;
        double invK = 1.0 / scaled_k_max;
        double C = cos(scaled_th0) - cos(scaled_thf);
        double S = 2 * scaled_k_max - sin(scaled_th0) + sin(scaled_thf);
        double temp1 = atan2(C, S);
        s1 = invK * mod2pi(scaled_th0 - temp1);
        double temp2 = 2 + 4 * pow(scaled_k_max, 2) - 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf));
        if (temp2 < 0)
        {
            s1 = 0;
            s2 = 0;
            s3 = 0;
            return new CurveSegmentsResult(false, s1, s2, s3);
        }
        s2 = invK * sqrt(temp2);
        s3 = invK * mod2pi(temp1 - scaled_thf);
        return new CurveSegmentsResult(true, s1, s2, s3);
    }

    CurveSegmentsResult *Dubins::curveLSR(double scaled_th0, double scaled_thf, double scaled_k_max)
    {
        double s1, s2, s3;
        double invK = 1.0 / scaled_k_max;
        double C = cos(scaled_th0) + cos(scaled_thf);
        double S = 2 * scaled_k_max + sin(scaled_th0) + sin(scaled_thf);
        double temp1 = atan2(-C, S);
        double temp3 = 4 * pow(scaled_k_max, 2) - 2 + 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) + sin(scaled_thf));
        if (temp3 < 0)
        {
            s1 = 0;
            s2 = 0;
            s3 = 0;
            return new CurveSegmentsResult(false, s1, s2, s3);
        }
        s2 = invK * sqrt(temp3);
        double temp2 = -atan2(-2, s2 * scaled_k_max);
        s1 = invK * mod2pi(temp1 + temp2 - scaled_th0);
        s3 = invK * mod2pi(temp1 + temp2 - scaled_thf);
        return new CurveSegmentsResult(true, s1, s2, s3);
    }

    CurveSegmentsResult *Dubins::curveRSL(double scaled_th0, double scaled_thf, double scaled_k_max)
    {
        double s1, s2, s3;
        double invK = 1.0 / scaled_k_max;
        double C = cos(scaled_th0) + cos(scaled_thf);
        double S = 2 * scaled_k_max - sin(scaled_th0) - sin(scaled_thf);
        double temp1 = atan2(C, S);
        double temp3 = 4 * pow(scaled_k_max, 2) - 2 + 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) + sin(scaled_thf));
        if (temp3 < 0)
        {
            s1 = 0;
            s2 = 0;
            s3 = 0;
            return new CurveSegmentsResult(false, s1, s2, s3);
        }
        s2 = invK * sqrt(temp3);
        double temp2 = atan2(2, s2 * scaled_k_max);
        s1 = invK * mod2pi(scaled_th0 - temp1 + temp2);
        s3 = invK * mod2pi(scaled_thf - temp1 + temp2);
        return new CurveSegmentsResult(true, s1, s2, s3);
    }

    CurveSegmentsResult *Dubins::curveRLR(double scaled_th0, double scaled_thf, double scaled_k_max)
    {
        double s1, s2, s3;
        double invK = 1.0 / scaled_k_max;
        double C = cos(scaled_th0) - cos(scaled_thf);
        double S = 2 * scaled_k_max - sin(scaled_th0) + sin(scaled_thf);
        double temp1 = atan2(C, S);
        double temp2 = 0.125 * (6 - 4 * pow(scaled_k_max, 2) + 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf)));
        if (abs(temp2) > 1)
        {
            s1 = 0;
            s2 = 0;
            s3 = 0;
            return new CurveSegmentsResult(false, s1, s2, s3);
        }
        s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        s1 = invK * mod2pi(scaled_th0 - temp1 + 0.5 * s2 * scaled_k_max);
        s3 = invK * mod2pi(scaled_th0 - scaled_thf + scaled_k_max * (s2 - s1));
        return new CurveSegmentsResult(true, s1, s2, s3);
    }

    CurveSegmentsResult *Dubins::curveLRL(double scaled_th0, double scaled_thf, double scaled_k_max)
    {
        double s1, s2, s3;
        double invK = 1.0 / scaled_k_max;
        double C = cos(scaled_thf) - cos(scaled_th0);
        double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);
        double temp1 = atan2(C, S);
        double temp2 = 0.125 * (6 - 4 * pow(scaled_k_max, 2) + 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf)));
        if (abs(temp2) > 1)
        {
            s1 = 0;
            s2 = 0;
            s3 = 0;
            return new CurveSegmentsResult(false, s1, s2, s3);
        }
        s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        s1 = invK * mod2pi(temp1 - scaled_th0 + 0.5 * s2 * scaled_k_max);
        s3 = invK * mod2pi(scaled_thf - scaled_th0 + scaled_k_max * (s2 - s1));
        return new CurveSegmentsResult(true, s1, s2, s3);
    }

    /**
     * @brief Find the shortest path between a starting and a final position
     *
     * @param x0 Starting x position
     * @param y0 Starting y position
     * @param th0 Starting angle
     * @param xf Final x position
     * @param yf Final y position
     * @param thf Final angle
     * @return Curve* Resulting curve representing the shortest path
     */
    Curve *Dubins::findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf)
    {
        DubinsResult *scaled_parameters = scaleToStandard(x0, y0, th0, xf, yf, thf);

        Curve *curve = nullptr;
        CurveSegmentsResult *best_curve_segments = nullptr;
        CurveSegmentsResult *curve_segments = nullptr;

        double best_L = DBL_MAX;
        int pidx = -1;

        for (int i = 0; i < TOTAL_POSSIBLE_CURVES; i++)
        {
            switch (i)
            {
                case LSL:
                    curve_segments = curveLSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case RSR:
                    curve_segments = curveRSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case LSR:
                    curve_segments = curveLSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case RSL:
                    curve_segments = curveRSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case RLR:
                    curve_segments = curveRLR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case LRL:
                    curve_segments = curveLRL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                default:
                    curve_segments = new CurveSegmentsResult(false, 0, 0, 0);
                    break;
            }

            double current_L = curve_segments->s1 + curve_segments->s2 + curve_segments->s3;

            if (curve_segments->ok && current_L < best_L)
            {
                best_L = current_L;
                pidx = i;
                if (best_curve_segments != nullptr)
                    delete best_curve_segments;
                best_curve_segments = new CurveSegmentsResult(true, curve_segments->s1, curve_segments->s2, curve_segments->s3);
            }
            delete curve_segments;
            curve_segments = nullptr;
        }

        if (pidx >= 0)
        {
            CurveSegmentsResult *curve_result = scaleFromStandard(scaled_parameters->lambda, best_curve_segments);

            curve = new Curve(x0, y0, th0, curve_result->s1, curve_result->s2, curve_result->s3, ksigns[pidx][0] * k_max, ksigns[pidx][1] * k_max, ksigns[pidx][2] * k_max);

            bool valid = areCurvesValid(best_curve_segments, ksigns[pidx][0] * scaled_parameters->scaled_k_max, ksigns[pidx][1] * scaled_parameters->scaled_k_max, ksigns[pidx][2] * scaled_parameters->scaled_k_max, scaled_parameters->scaled_th0, scaled_parameters->scaled_thf);
            if (!valid)
            {
                std::cout << "NOT VALID!!\n\n";
                delete curve;
                curve = nullptr;
            }
            delete curve_result;
            curve_result = nullptr;
        }
        delete scaled_parameters;
        delete best_curve_segments;
        return curve;
    };

    /**
     * @brief Find if there is an intersection between two segments
     *
     * @param p1 First point used to define the first segment
     * @param p2 Second point used to define the first segment
     * @param p3 First point used to define the second segment
     * @param p4 Second point used to define the second segment
     * @param pts Array of intersection points this function has found (passed by ref.)
     * @param ts Coefficients to normalize the segments (passed by ref.)
     * @return true If an intersection has been found
     * @return false If an intersection has not been found
     */
    bool Dubins::lineLineIntersection(DubinsPoint p1, DubinsPoint p2, DubinsPoint p3, DubinsPoint p4, std::vector<DubinsPoint> &pts, std::vector<double> &ts)
    {
        const double EPSILON = 0.0000001;
        // Initialize the resulting arrays as empty arrays
        pts.clear();
        ts.clear();

        // Define min and max coordinates of the first segment
        double minX1 = std::min(p1.x, p2.x);
        double minY1 = std::min(p1.y, p2.y);
        double maxX1 = std::max(p1.x, p2.x);
        double maxY1 = std::max(p1.y, p2.y);

        // Define min and max coordinates of the second segment
        double minX2 = std::min(p3.x, p4.x);
        double minY2 = std::min(p3.y, p4.y);
        double maxX2 = std::max(p3.x, p4.x);
        double maxY2 = std::max(p3.y, p4.y);

        // If the coordinates of the two segments do not overlap, return false
        if (maxX2 < minX1 || minX2 > maxX1 || maxY2 < minY1 || minY2 > maxY1)
        {
            return false;
        }

        DubinsPoint s = DubinsPoint((p2.x - p1.x), (p2.y - p1.y));

        DubinsPoint p = DubinsPoint(p3.x, p3.y);
        DubinsPoint r = DubinsPoint((p4.x - p3.x), (p4.y - p3.y));

        DubinsPoint diffPQ = DubinsPoint((p1.x - p3.x), (p1.y - p3.y));

        double crossRS = crossProduct(r, s);
        double crossDiffR = crossProduct(diffPQ, r);
        double crossDiffS = crossProduct(diffPQ, s);

        if (crossRS == 0 && crossDiffR == 0)
        {
            double dotRR = dot2D(r, r);
            double dotSR = dot2D(s, r);
            double t0 = dot2D(diffPQ, r) / dotRR;
            double t1 = t0 + (dotSR / dotRR);
            if (dotSR < 0)
            {
                if (t0 >= 0-EPSILON && t1 <= 1+EPSILON)
                {
                    ts.push_back(std::max(t1, 0.0));
                    ts.push_back(std::min(t0, 1.0));
                }
            }
            else
            {
                if (t1 >= 0-EPSILON && t0 <= 1+EPSILON)
                {
                    ts.push_back(std::max(t0, 0.0));
                    ts.push_back(std::min(t1, 1.0));
                }
            }
        }
        else
        {
            if (crossRS == 0 && crossDiffR != 0)
            {
                return false;
            }
            else
            {
                double t = crossDiffS / crossRS;
                double u = crossDiffR / crossRS;
                if ((t >= 0-EPSILON && t <= 1+EPSILON && u >= 0-EPSILON && u <= 1+EPSILON))
                {
                    ts.push_back(t);
                }
            }
        }
        for (std::vector<double>::size_type i = 0; i < ts.size(); i++)
        {
            pts.push_back(DubinsPoint((ts[i] * r.x) + p.x, (ts[i] * r.y) + p.y));
        }

        return pts.empty() ? false : true;
    }

    bool Dubins::arcLineIntersection(Arc *arc, DubinsPoint point1, DubinsPoint point2, std::vector<DubinsPoint> &pts, std::vector<double> &t)
    {
        const double EPSILON = 0.0000001;
        // Find the circle containing the provided arc
        // Get the perpendicular lines of point1's line and points2's line
        // (we can calculate them because point1 and point2 also contain the angle, that in our case will be the slope)
        // Their intersection will be the center of the circle

        double tanFirstAngle = tan(arc->th0);
        double tanSecondAngle = tan(arc->dubins_line->th);

        double m1 = tanFirstAngle == INFINITY ? 0 : (-1 / tanFirstAngle);
        double m2 = tanSecondAngle == INFINITY ? 0 : (-1 / tanSecondAngle);


        if (tanFirstAngle > 500 || tanFirstAngle < -500)
            m1 = 0;
        else if (tanSecondAngle > 500 || tanSecondAngle < -500)
            m2 = 0;

        if (abs(tanFirstAngle) <=  0+EPSILON)
            m1 = INFINITY;
        if (abs(tanSecondAngle) <= 0+EPSILON)
            m2 = INFINITY;

        DubinsPoint circleCenter = DubinsPoint(-1, -1);

        // Limit case: the slope is the same
        if (abs(m1 - m2) < 1.e-1 || abs(m1 + m2) < 1.e-1)
        {
            if (arc->x0 == arc->dubins_line->x && arc->y0 == arc->dubins_line->y)
            {
                // The two points are the same, no intersection?
                return false;
            }
            else
            {
                // The segment between the two points forms the diagonal of the circle
                // This means the center is in the middle
                circleCenter.x = (arc->x0 + arc->dubins_line->x) / 2;
                circleCenter.y = (arc->y0 + arc->dubins_line->y) / 2;
            }
        }
        else
        {
            // Intersection between the two perpendicular lines
            if (m1 == INFINITY) {
                double q2 = arc->dubins_line->y - (m2*arc->dubins_line->x);
                circleCenter.x = arc->x0;
                circleCenter.y = m2 * circleCenter.x + q2;
            } else if (m2 == INFINITY) {
                double q1 = arc->y0 - (m1*arc->x0);
                circleCenter.x = arc->dubins_line->x;
                circleCenter.y = m1 * circleCenter.x + q1;
            } else {
                double q1 = arc->y0 - (m1*arc->x0);
                double q2 = arc->dubins_line->y - (m2*arc->dubins_line->x);
                circleCenter.x = (q2 - q1) / (m1 - m2);
                circleCenter.y = m1 * circleCenter.x + q1;
            }
        }

        // Having the center, we can easily find the radius
        double r = sqrt(pow(arc->x0 - circleCenter.x, 2) + pow(arc->y0 - circleCenter.y, 2));

        // Initialize the resulting arrays as empty arrays
        pts.clear();
        t.clear();

        double p1 = 2 * point1.x * point2.x;
        double p2 = 2 * point1.y * point2.y;
        double p3 = 2 * circleCenter.x * point1.x;
        double p4 = 2 * circleCenter.x * point2.x;
        double p5 = 2 * circleCenter.y * point1.y;
        double p6 = 2 * circleCenter.y * point2.y;

        double c1 = pow(point1.x, 2) + pow(point2.x, 2) - p1 + pow(point1.y, 2) + pow(point2.y, 2) - p2;
        double c2 = -2 * pow(point2.x, 2) + p1 - p3 + p4 - 2 * pow(point2.y, 2) + p2 - p5 + p6;
        double c3 = pow(point2.x, 2) - p4 + pow(circleCenter.x, 2) + pow(point2.y, 2) - p6 + pow(circleCenter.y, 2) - pow(r, 2);

        double delta = pow(c2, 2) - (4 * c1 * c3);

        double t1;
        double t2;

        if (delta < 0)
        {
            // There is no solution (we are not dealing with complex numbers)
            return false;
        }
        else
        {
            if (delta > 0)
            {
                // Two points of intersection
                double deltaSq = sqrt(delta);
                t1 = (-c2 + deltaSq) / (2 * c1);
                t2 = (-c2 - deltaSq) / (2 * c1);
            }
            else
            {
                // If the delta is 0 we have just one point of intersection
                t1 = -c2 / (2 * c1);
                t2 = t1;
            }
        }

        std::vector<std::pair<DubinsPoint, double>> intersections = std::vector<std::pair<DubinsPoint, double>>();

        if (t1 >= 0 && t1 <= 1)
        {
            intersections.push_back(std::pair<DubinsPoint, double>(DubinsPoint((point1.x * t1) + (point2.x * (1 - t1)), (point1.y * t1) + (point2.y * (1 - t1))), t1));
        }

        if (t2 >= 0 && t2 <= 1 && t2 != t1)
        {
            intersections.push_back(std::pair<DubinsPoint, double>(DubinsPoint((point1.x * t2) + (point2.x * (1 - t2)), (point1.y * t2) + (point2.y * (1 - t2))), t2));
        }

        // Sort the intersections using t values
        std::sort(intersections.begin(), intersections.end(), [](const std::pair<DubinsPoint, double> a, const std::pair<DubinsPoint, double> b)
        { return a.second < b.second; });

        // Check if the arc goes clockwise (orientation=-1) or counterclockwise(orientation=1)
        double s = arc->L/100 * 50;
        Line *tmp = new Line(s, arc->x0, arc->y0, arc->th0, arc->k);
        Point middlePoint = Point(tmp->x, tmp->y);
        int orientation = getOrientation(Point(arc->x0, arc->y0), middlePoint, Point(arc->dubins_line->x, arc->dubins_line->y));
        delete tmp;

        // Fill the resulting arrays
        for (std::vector<std::pair<dubins::DubinsPoint, double> >::size_type i = 0; i < intersections.size(); i++)
        {
            // Check if the intersection is inside the arc provided at the beginning
            double intersectionTh = (atan2(intersections[i].first.y - circleCenter.y, intersections[i].first.x - circleCenter.x));
            double firstTh = (atan2(arc->y0 - circleCenter.y, arc->x0 - circleCenter.x));
            double secondTh = (atan2(arc->dubins_line->y - circleCenter.y, arc->dubins_line->x - circleCenter.x));

            if (orientation == 1) {
                // Counter Clockwise Arc
                if (firstTh < secondTh && (intersectionTh >= firstTh && intersectionTh <= secondTh)) {
                    pts.push_back(intersections[i].first);
                    t.push_back(intersections[i].second);
                } else if (firstTh > secondTh) {
                    if (intersectionTh >= firstTh || intersectionTh <= secondTh) {
                        pts.push_back(intersections[i].first);
                        t.push_back(intersections[i].second);
                    }
                }
            } else {
                // Clockwise Arc
                if (firstTh < secondTh) {
                    if (intersectionTh <= firstTh || intersectionTh >= secondTh) {
                        pts.push_back(intersections[i].first);
                        t.push_back(intersections[i].second);
                    }
                } else if (firstTh > secondTh && (intersectionTh <= firstTh && intersectionTh >= secondTh)) {
                    pts.push_back(intersections[i].first);
                    t.push_back(intersections[i].second);
                }
            }
        }

        return pts.empty() ? false : true;
    }

    double* Dubins::getAnglesShortestPath(DubinsPoint **points, int numberOfPoints, std::vector<Polygon> obstacles, Polygon map)
    {
        // INITIALIZATION
        // Set up a vector of the desired angles
        double *minimizingAngles = new double[numberOfPoints];

        // Get the length of the array of the discretized angles
        const int K = std::extent<decltype(multipointAngles)>::value;

        // Set up the L matrix, used for the dynamic programming algorithm, storing the intermediate lengths.
        // For each point, there are K possible angles, so a matrix of size numberOfPoints x K.
        double **L = new double *[numberOfPoints];
        for (int i = 0; i < numberOfPoints; i++)
        {
            L[i] = new double[K];
        }
        // The length of the path is initialized to 0 for the last point, the others are set to -1.
        for (int n = 0; n < numberOfPoints; n++)
        {
            for (unsigned int i = 0; i < K; i++)
            {
                if (n == numberOfPoints - 1)
                    L[n][i] = 0;
                else
                    L[n][i] = -1;
            }
        }

        // Set up matrix S that defines which angle to finish based on the L matrix.
        // Used to recreate the complete solution after having discovered which is the best path
        int **S = new int *[numberOfPoints];
        for (int i = 0; i < numberOfPoints; i++)
        {
            S[i] = new int[K];
        }
        // Fill the matrix with -1
        for (int n = 0; n < numberOfPoints; n++)
        {
            for (unsigned int i = 0; i < K; i++)
            {
                S[n][i] = -1;
            }
        }

        // ALGORITHM - FIRST STEP
        // For the last two points, we already know the end angle
        bool isThereAPath = false;
        // Loop over the K angles. For each of them, find a curve. If there is a curve, store the length in the L matrix.
        for (unsigned int i = 0; i < K; i++)
        {
            Curve *curve = ShortestPathWithoutCollisions(points[numberOfPoints - 2]->x, points[numberOfPoints - 2]->y, multipointAngles[i], points[numberOfPoints - 1]->x, points[numberOfPoints - 1]->y, points[numberOfPoints - 1]->th, obstacles, map);
            if (curve != nullptr) {
                isThereAPath = true;
                // Set the curve length if it is shorter than the current one or if the value is set to -1.
                if (L[numberOfPoints - 2][i] == -1 || L[numberOfPoints - 2][i] > curve->L)
                {
                    L[numberOfPoints - 2][i] = curve->L;
                }
                delete curve;
            } else {
                // If there is no curve, set the length to infinity, to describe that there is no path.
                L[numberOfPoints - 2][i] = INFINITY;
            }
        }
        // If it is not possible to find a path between the last two points, a curve cannot be defined.
        if (!isThereAPath) {
            std::cout << "NOT FOUND A PATH FOR LAST CURVE\n";
            return nullptr;
        }

        // ITERATIVE COMPUTATION
        // Loop over the points, starting from the second last one, until the first one.
        for (int n = numberOfPoints - 3; n >= 0; n--) {
            // Since there is no constraint on the starting angle, loop over all the K angles.
            for (unsigned int i = 0; i < K; i++) {
                // Loop over the K angles.
                for (unsigned int j = 0; j < K; j++) {
                    // Try to find a path only if the length of the path from the next point to the end is not infinite.
                    // I.e., if this path has a valid curve.
                    if (L[n + 1][j] != INFINITY) {
                        Curve *curve = ShortestPathWithoutCollisions(points[n]->x, points[n]->y,
                                                                                multipointAngles[i], points[n + 1]->x,
                                                                                points[n + 1]->y, multipointAngles[j],
                                                                                obstacles, map);
                        if (curve != nullptr) {
                            // A valid curve has been found. Check if it is shorter than the current one.
                            if (L[n][i] > curve->L + L[n + 1][j] || L[n][i] == -1) {
                                // Update the length and select which angle to take in that configuration.
                                L[n][i] = curve->L + L[n + 1][j];
                                S[n][i] = j;
                            }
                            delete curve;
                        } else {
                            if (L[n][i] == -1) {
                                L[n][i] = INFINITY;
                                S[n][i] = -1;
                            }
                        }
                    }
                }
            }
        }

        // Find the minimum length and the corresponding index in the first row of L.
        // The two matrices have been filled. Reconstruct the solution.
        int minIndex = -1;
        double minLength = INFINITY;
        // Find the first best angle by looking at the first row of L.
        for (unsigned int i = 0; i < K; i++)
        {
            if (L[0][i] < minLength)
            {
                minIndex = i;
                minLength = L[0][i];
            }
        }

        if (minIndex == -1) {
            return nullptr;
        }

        // Get the angle.
        minimizingAngles[0] = multipointAngles[minIndex];
        // Use the S matrix to find the other angles. In the S matrix the index has been stored.
        for (int i = 0; i < numberOfPoints - 2; i++)
        {
            minimizingAngles[i + 1] = multipointAngles[S[i][minIndex]];
            minIndex = S[i][minIndex];
        }
        minimizingAngles[numberOfPoints - 1] = points[numberOfPoints - 1]->th;

        return minimizingAngles;
    };

    // TODO: understand if a vector can be returned as below
    std::vector<std::vector<Point>> Dubins::getObstaclesAndMapLines(std::vector<Polygon> obstacles, Polygon map) {
        std::vector<std::vector<Point>> obstaclesLines;
        for (std::vector<Polygon>::size_type i = 0; i < obstacles.size(); i++) {
            for (std::vector<Point> edge: obstacles[i].getPolygonEdges()) {
                obstaclesLines.push_back(edge);
            }
        }
        for(std::vector<Point> edge: map.getPolygonEdges()) {
            obstaclesLines.push_back(edge);
        }
        return obstaclesLines;
    }

    /**
     * @brief Find the shortest path between a starting and a final position, check for collisions
     *
     * @param x0 Starting x position
     * @param y0 Starting y position
     * @param th0 Starting angle
     * @param xf Final x position
     * @param yf Final y position
     * @param thf Final angle
     * @param edges All obstacles' edges
     * @return Curve* Resulting curve representing the shortest path
     */
    Curve *Dubins::ShortestPathWithoutCollisions(double x0, double y0, double th0, double xf, double yf, double thf, std::vector<Polygon> obstacles, Polygon map)
    {
        // Get the obstacles and the map edges.
        std::vector<std::vector<Point>> obstaclesLines = getObstaclesAndMapLines(obstacles, map);
        // Scale the parameters to the standard case (easier to work with in dubins).
        DubinsResult *scaled_parameters = scaleToStandard(x0, y0, th0, xf, yf, thf);

        Curve *curve = nullptr;
        CurveSegmentsResult *best_curve_segments = nullptr;
        CurveSegmentsResult *curve_segments = nullptr;

        double best_L = DBL_MAX;
        int pidx = -1;
        // Try each of the possible curves.
        for (int i = 0; i < TOTAL_POSSIBLE_CURVES; i++)
        {
            bool isThereAStraightLine = false;
            switch (i)
            {
                case LSL:
                    curve_segments = curveLSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);
                    isThereAStraightLine = true;

                    break;

                case RSR:
                    curve_segments = curveRSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);
                    isThereAStraightLine = true;

                    break;

                case LSR:
                    curve_segments = curveLSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);
                    isThereAStraightLine = true;

                    break;

                case RSL:
                    curve_segments = curveRSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);
                    isThereAStraightLine = true;

                    break;

                case RLR:
                    curve_segments = curveRLR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case LRL:
                    curve_segments = curveLRL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                default:
                    curve_segments = new CurveSegmentsResult(false, 0, 0, 0);
                    break;
            }
            // Define the current length
            double current_L = curve_segments->s1 + curve_segments->s2 + curve_segments->s3;
            // Rescale the curve segments to the original case.
            CurveSegmentsResult *tmpCurveResult = scaleFromStandard(scaled_parameters->lambda, curve_segments);

            // Create the curve associated to the one found.
            Curve *tmpCurve = new Curve(x0, y0, th0, tmpCurveResult->s1, tmpCurveResult->s2, tmpCurveResult->s3, ksigns[i][0] * k_max, ksigns[i][1] * k_max, ksigns[i][2] * k_max);
            // If the curve found is shorter than the previous one, check for collisions.
            if (curve_segments->ok && current_L < best_L)
            {
                bool areThereCollisions = false;
                std::vector<DubinsPoint> polTmp;
                std::vector<double> tTmp;
                // For each obstacle edge.
                for (std::vector<std::vector<Point> >::size_type z = 0; z < obstaclesLines.size(); z++) {
                    // Check if the first arc intersects with the obstacle z.
                    areThereCollisions = areThereCollisions || arcLineIntersection(tmpCurve->a1, DubinsPoint(obstaclesLines[z][0].x, obstaclesLines[z][0].y), DubinsPoint(obstaclesLines[z][1].x, obstaclesLines[z][1].y), polTmp, tTmp);
                    // If the second curve is a straight line, check if it intersects with one of the obstacles, otherwise use the arc check.
                    if (isThereAStraightLine)
                        areThereCollisions = areThereCollisions || lineLineIntersection(DubinsPoint(tmpCurve->a2->x0, tmpCurve->a2->y0), DubinsPoint(tmpCurve->a2->dubins_line->x, tmpCurve->a2->dubins_line->y),DubinsPoint(obstaclesLines[z][0].x, obstaclesLines[z][0].y), DubinsPoint(obstaclesLines[z][1].x, obstaclesLines[z][1].y), polTmp, tTmp);
                    else
                        areThereCollisions = areThereCollisions || arcLineIntersection(tmpCurve->a2, DubinsPoint(obstaclesLines[z][0].x, obstaclesLines[z][0].y), DubinsPoint(obstaclesLines[z][1].x, obstaclesLines[z][1].y), polTmp, tTmp);
                    // Check if the third arc intersects with the obstacle z.
                    areThereCollisions = areThereCollisions || arcLineIntersection(tmpCurve->a3, DubinsPoint(obstaclesLines[z][0].x, obstaclesLines[z][0].y), DubinsPoint(obstaclesLines[z][1].x, obstaclesLines[z][1].y), polTmp, tTmp);
                }
                // If there are no collisions, update the best curve.
                if (!areThereCollisions) {
                    best_L = current_L;
                    pidx = i;
                    if (best_curve_segments != nullptr) {
                        delete best_curve_segments;
                        best_curve_segments = nullptr;
                    }
                    best_curve_segments = new CurveSegmentsResult(true, curve_segments->s1, curve_segments->s2, curve_segments->s3);
                }
            }
            delete tmpCurveResult;
            delete tmpCurve;
            delete curve_segments;
            curve_segments = nullptr;
        }
        // If at least one curve has no collisions.
        if (pidx >= 0)
        {
            CurveSegmentsResult *curve_result = scaleFromStandard(scaled_parameters->lambda, best_curve_segments);

            curve = new Curve(x0, y0, th0, curve_result->s1, curve_result->s2, curve_result->s3, ksigns[pidx][0] * k_max, ksigns[pidx][1] * k_max, ksigns[pidx][2] * k_max);
            // Check if the curve is valid.
            bool valid = areCurvesValid(best_curve_segments, ksigns[pidx][0] * scaled_parameters->scaled_k_max, ksigns[pidx][1] * scaled_parameters->scaled_k_max, ksigns[pidx][2] * scaled_parameters->scaled_k_max, scaled_parameters->scaled_th0, scaled_parameters->scaled_thf);
            if (!valid)
            {
                delete curve;
                curve = nullptr;
            }
            delete curve_result;
            curve_result = nullptr;
        }
        delete scaled_parameters;
        delete best_curve_segments;
        return curve;
    };

    Curve **Dubins::multipointShortestPath(DubinsPoint **points, int numberOfPoints, std::vector<Polygon> obstacles, Polygon map)
    {
        if (numberOfPoints > 1) {
            DubinsPoint **newPoints = new DubinsPoint*[numberOfPoints];
            for (int i = 0; i < numberOfPoints; i++) {
                newPoints[i] = points[numberOfPoints - i - 1];
            }
            newPoints[numberOfPoints-1]->th = mod2pi(points[0]->th + M_PI);
            // Get the optimal angles for each point (dynamic programming iterative procedure)
            double *angles = getAnglesShortestPath(newPoints, numberOfPoints, obstacles, map);
            // If no valid angles have been computed, return nullptr
            if (angles == nullptr) {
                delete[] newPoints;
                return nullptr;
            }

            // The best angles have been defined, compute the optimal multipoint shortest pathShortestPathWithoutCollisions
            Curve **curves = new Curve*[numberOfPoints-1];
            // For each point
            for (int i = 0; i < numberOfPoints-1; i++) {
                int index = numberOfPoints-i-1;
                // Compute the curves starting from the initial point (the last point of the array, which is filled in the reverse order to find the optimal angles),
                // passing the best angle found.
                curves[i] = ShortestPathWithoutCollisions(newPoints[index]->x, newPoints[index]->y, mod2pi(angles[index] + M_PI), newPoints[index-1]->x, newPoints[index-1]->y, mod2pi(angles[index-1] + M_PI), obstacles, map);
                if (curves[i] == nullptr) {
                    delete[] newPoints;
                    return nullptr;
                }
            }
            delete[] newPoints;
            return curves;
        }
        return nullptr;
    }
}