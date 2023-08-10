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

#include <iostream>
#include <cfloat>
#include <algorithm>

using namespace cv;

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
     * @brief Check the validity of a provided Dubins solution
     *
     * @param curve_segments Resulting curve segments representing the final solution
     * @param k0 TODO
     * @param k1 TODO
     * @param k2 TODO
     * @param th0 Starting angle
     * @param thf Final angle
     * @return true If the solution is valid
     * @return false If the solution is not valid
     */
    bool Dubins::checkValidity(CurveSegmentsResult *curve_segments, double k0, double k1, double k2, double th0, double thf)
    {
        double x0 = -1;
        double y0 = 0;
        double xf = 1;
        double yf = 0;

        double eq1 = x0 + curve_segments->s1 * sinc((1 / 2.) * k0 * curve_segments->s1) * cos(th0 + (1 / 2.) * k0 * curve_segments->s1) + curve_segments->s2 * sinc((1 / 2.) * k1 * curve_segments->s2) * cos(th0 + k0 * curve_segments->s1 + (1 / 2.) * k1 * curve_segments->s2) + curve_segments->s3 * sinc((1 / 2.) * k2 * curve_segments->s3) * cos(th0 + k0 * curve_segments->s1 + k1 * curve_segments->s2 + (1 / 2.) * k2 * curve_segments->s3) - xf;
        double eq2 = y0 + curve_segments->s1 * sinc((1 / 2.) * k0 * curve_segments->s1) * sin(th0 + (1 / 2.) * k0 * curve_segments->s1) + curve_segments->s2 * sinc((1 / 2.) * k1 * curve_segments->s2) * sin(th0 + k0 * curve_segments->s1 + (1 / 2.) * k1 * curve_segments->s2) + curve_segments->s3 * sinc((1 / 2.) * k2 * curve_segments->s3) * sin(th0 + k0 * curve_segments->s1 + k1 * curve_segments->s2 + (1 / 2.) * k2 * curve_segments->s3) - yf;
        double eq3 = rangeSymm(k0 * curve_segments->s1 + k1 * curve_segments->s2 + k2 * curve_segments->s3 + th0 - thf);

        bool Lpos = (curve_segments->s1 > 0) || (curve_segments->s2 > 0) || (curve_segments->s3 > 0);
        // TODO: I modified the value 1.e-10 to 1.e-2 -> if we lower the threshold, the program says our results are valid.
        // otherwise not. This is the problem I was describing in the group
        return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-2 && Lpos);
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
     * @return ParametersResult* Scaled parameters
     */
    ParametersResult *Dubins::scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf)
    {
        double dx = xf - x0;
        double dy = yf - y0;
        double phi = atan2(dy, dx);
        double lambda = hypot(dx, dy) / 2;

        double sc_th0 = mod2pi(th0 - phi);
        double sc_thf = mod2pi(thf - phi);
        double sc_k_max = k_max * lambda;
        return new ParametersResult(sc_th0, sc_thf, sc_k_max, lambda);
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

    CurveSegmentsResult *Dubins::useLSL(double scaled_th0, double scaled_thf, double scaled_k_max)
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

    CurveSegmentsResult *Dubins::useRSR(double scaled_th0, double scaled_thf, double scaled_k_max)
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

    CurveSegmentsResult *Dubins::useLSR(double scaled_th0, double scaled_thf, double scaled_k_max)
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

    CurveSegmentsResult *Dubins::useRSL(double scaled_th0, double scaled_thf, double scaled_k_max)
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

    CurveSegmentsResult *Dubins::useRLR(double scaled_th0, double scaled_thf, double scaled_k_max)
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

    CurveSegmentsResult *Dubins::useLRL(double scaled_th0, double scaled_thf, double scaled_k_max)
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
     * @return DubinsCurve* Resulting curve representing the shortest path
     */
    DubinsCurve *Dubins::findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf)
    {
        ParametersResult *scaled_parameters = scaleToStandard(x0, y0, th0, xf, yf, thf);

        DubinsCurve *curve = nullptr;
        CurveSegmentsResult *best_curve_segments = nullptr;
        CurveSegmentsResult *curve_segments = nullptr;

        double best_L = DBL_MAX;
        int pidx = -1;

        for (int i = 0; i < TOTAL_POSSIBLE_CURVES; i++)
        {
            switch (i)
            {
                case LSL:
                    curve_segments = useLSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case RSR:
                    curve_segments = useRSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case LSR:
                    curve_segments = useLSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case RSL:
                    curve_segments = useRSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case RLR:
                    curve_segments = useRLR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

                    break;

                case LRL:
                    curve_segments = useLRL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

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

        bool valid = false;
        if (pidx >= 0)
        {
            CurveSegmentsResult *curve_result = scaleFromStandard(scaled_parameters->lambda, best_curve_segments);

            curve = new DubinsCurve(x0, y0, th0, curve_result->s1, curve_result->s2, curve_result->s3, ksigns[pidx][0] * k_max, ksigns[pidx][1] * k_max, ksigns[pidx][2] * k_max);

            bool valid = checkValidity(best_curve_segments, ksigns[pidx][0] * scaled_parameters->scaled_k_max, ksigns[pidx][1] * scaled_parameters->scaled_k_max, ksigns[pidx][2] * scaled_parameters->scaled_k_max, scaled_parameters->scaled_th0, scaled_parameters->scaled_thf);
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
    bool Dubins::intersLineLine(DubinsPoint p1, DubinsPoint p2, DubinsPoint p3, DubinsPoint p4, std::vector<DubinsPoint> &pts, std::vector<double> &ts)
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

        // If there is no way these segments will intersect, we just return false without computing anything
        if (maxX2 < minX1 || minX2 > maxX1 || maxY2 < minY1 || minY2 > maxY1)
        {
            return false;
        }

        DubinsPoint q = DubinsPoint(p1.x, p1.y);
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
        for (int i = 0; i < ts.size(); i++)
        {
            pts.push_back(DubinsPoint((ts[i] * r.x) + p.x, (ts[i] * r.y) + p.y));
        }

        return pts.empty() ? false : true;
    }

    /**
     * @brief Find if there is an intersection between a circle and a segment
     *
     * @param circleCenter Center of the input circle
     * @param r Radius of the input circle
     * @param point1 First point used to define the segment
     * @param point2 Second point used to define the segment
     * @param pts Array of intersection points this function has found (passed by ref.)
     * @param t Coefficient to normalize the segment (passed by ref.)
     * @return true If an intersection has been found
     * @return false If an intersection has not been found
     */
    bool Dubins::intersCircleLine(DubinsPoint circleCenter, double r, DubinsPoint point1, DubinsPoint point2, std::vector<DubinsPoint> &pts, std::vector<double> &t)
    {
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

        // Fill the resulting arrays
        for (int i = 0; i < intersections.size(); i++)
        {
            pts.push_back(intersections[i].first);
            t.push_back(intersections[i].second);
        }

        return pts.empty() ? false : true;
    }



    /**
     * @brief Helper function to plot with opencv a Dubins Arc
     *
     * @param arc DubinsArc to plot
     * @param image OpenCV Mat in which we want to plot
     * @param size Size of the Mat (to scale the points)
     * @param first Is this the first arc of a curve?
     * @param last Is this the last arc of a curve?
     */
    void Dubins::printDubinsArc(DubinsArc *arc, Mat image, double size, bool first, bool last) {
        int npts = 100;
        std::vector<cv::Point> pts;
        if (first)
            circle(image, cv::Point(arc->x0 / size * 500, arc->y0 / size * 500), 5, Scalar(255, 255, 255), FILLED, LINE_8);
        if (last)
            circle(image, cv::Point(arc->dubins_line->x / size * 500, arc->dubins_line->y / size * 500), 5, Scalar(255, 255, 255), FILLED, LINE_8);
        for (int i = 0; i < npts; i++) {
            double s = arc->L/npts * i;
            DubinsLine *tmp = new DubinsLine(s, arc->x0, arc->y0, arc->th0, arc->k);
            pts.push_back(cv::Point(tmp->x / size * 500, tmp->y / size * 500));
            delete tmp;
        }
        for (int i = 1; i < pts.size(); i++) {
            line(image, pts[i-1], pts[i], Scalar(0, 0, 255), 1, LINE_AA);
        }
    }

    /**
     * @brief Plot with opencv a Dubins Curve
     *
     * @param curve DubinsCurve to plot
     */
    void Dubins::printDubinsCurve(DubinsCurve *curve) {
        Mat image(500, 500, CV_8UC3, Scalar(0, 0, 0));
        Mat flipped;

        double maxx = max({curve->a1->x0, curve->a1->dubins_line->x, curve->a2->x0, curve->a2->dubins_line->x, curve->a3->x0, curve->a3->dubins_line->x});
        double minx = min({curve->a1->x0, curve->a1->dubins_line->x, curve->a2->x0, curve->a2->dubins_line->x, curve->a3->x0, curve->a3->dubins_line->x});
        double maxy = max({curve->a1->y0, curve->a1->dubins_line->y, curve->a2->y0, curve->a2->dubins_line->y, curve->a3->y0, curve->a3->dubins_line->y});
        double miny = min({curve->a1->y0, curve->a1->dubins_line->y, curve->a2->y0, curve->a2->dubins_line->y, curve->a3->y0, curve->a3->dubins_line->y});

        double size = max(maxx, maxy);

        printDubinsArc(curve->a1, image, size, true, false);
        printDubinsArc(curve->a2, image, size, false, false);
        printDubinsArc(curve->a3, image, size, false, true);

        flip(image, flipped, 0);
        imshow("Output", flipped);
        cv::waitKey(0);
    }
}