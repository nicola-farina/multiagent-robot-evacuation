#ifndef DUBINS_HPP
#define DUBINS_HPP

#include "utils.hpp"
#include "environment.hpp"
#include <vector>
#include <iostream>


namespace dubins {
    /**
     * @brief Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc length s
     */
    struct Line {
        double x, y, th;

        Line(double s, double x0, double y0, double th0, double k) {
            x = x0 + (s * sinc(k * s / 2.0) * cos(th0 + ((k * s) / 2)));
            y = y0 + (s * sinc(k * s / 2.0) * sin(th0 + ((k * s) / 2)));
            th = mod2pi(th0 + (k * s));
        }
    };

    /**
     * @brief Structure representing an arc of a Dubins curve (it can be either straight or circular)
     */
    struct Arc {
        double x0, y0, th0, k, L;
        Line *dubins_line;

        Arc(double i_x0, double i_y0, double i_th0, double i_k, double i_L) {
            dubins_line = new Line(i_L, i_x0, i_y0, i_th0, i_k);
            x0 = i_x0;
            y0 = i_y0;
            th0 = i_th0;
            k = i_k;
            L = i_L;
        }

        ~Arc() {
            delete dubins_line;
        }
    };

    /**
     * @brief Structure representing a Dubins curve (it is composed by three arcs)
     */
    struct Curve {
        Arc *a1;
        Arc *a2;
        Arc *a3;
        double L;

        Curve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2) {
            a1 = new Arc(x0, y0, th0, k0, s1);
            a2 = new Arc(a1->dubins_line->x, a1->dubins_line->y, a1->dubins_line->th, k1, s2);
            a3 = new Arc(a2->dubins_line->x, a2->dubins_line->y, a2->dubins_line->th, k2, s3);

            L = a1->L + a2->L + a3->L;
        }

        ~Curve() {
            delete a1;
            delete a2;
            delete a3;
        }
    };

    /**
     * @brief Structure representing the scaled parameters of the dubins problem
     */
    struct DubinsResult {
        double scaled_th0, scaled_thf, scaled_k_max, lambda;

        DubinsResult(double scaled_th0, double scaled_thf, double scaled_k_max, double lambda) : scaled_th0(scaled_th0), scaled_thf(scaled_thf), scaled_k_max(scaled_k_max), lambda(lambda) {}
    };

    /**
     * @brief Structure representing the solution of the problem, where s1, s2 and s3 are the lengths of the three arcs composing the curve.
     */
    struct CurveSegmentsResult {
        bool ok;
        double s1, s2, s3;

        CurveSegmentsResult(bool ok, double s1, double s2, double s3) : ok(ok), s1(s1), s2(s2), s3(s3) {}
    };

    /**
     * @brief Class used to find the best path between two or more points using dubins curves
     */
    class Dubins {
    public:
        /**
         * @brief Construct a new Dubins object
         *
         */
        Dubins() = default;

        /**
         * @brief Construct a new Dubins object
         *
         * @param k_max Bound on maximum path curvature
         * @param discritizer_size Given a path of infinite points, what is the discritizer size? Expressed in meters
         */
        explicit Dubins(double k_max, double discritizer_size);

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
        Curve *findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf);

        /**
         * @brief Find if there is an intersection between an arc and a segment
         *
         * @param arc Arc of a Dubins curve
         * @param point1 First point used to define the segment
         * @param point2 Second point used to define the segment
         * @param pts Array of intersection points this function has found (passed by ref.)
         * @param t Coefficient to normalize the segment (passed by ref.)
         * @return true If an intersection has been found
         * @return false If an intersection has not been found
         */
        static bool arcLineIntersection(Arc *arc, DubinsPoint point1, DubinsPoint point2, std::vector<DubinsPoint> &pts, std::vector<double> &t);

        /**
         * @brief Find if there is an intersection between two segments
         *
         * @param p1 First point used to define the first segment
         * @param p2 Second point used to define the first segment
         * @param p3 First point used to define the second segment
         * @param p4 Second point used to define the second segment
         * @param pts Array of intersection points this function has found (passed by ref.)
         * @param ts Coefficients to normalize the segment (passed by ref.)
         * @return true If an intersection has been found
         * @return false If an intersection has not been found
         */
        static bool lineLineIntersection(DubinsPoint p1, DubinsPoint p2, DubinsPoint p3, DubinsPoint p4, std::vector<DubinsPoint> &pts, std::vector<double> &ts);

        double *getAnglesShortestPath(DubinsPoint **points, int numberOfPoints, const std::vector<evacuation::Polygon> &obstacles, const evacuation::Polygon &map);

        static std::vector<std::vector<evacuation::Point>> getObstaclesAndMapLines(std::vector<evacuation::Polygon> obstacles, evacuation::Polygon map);

        Curve *ShortestPathWithoutCollisions(double x0, double y0, double th0, double xf, double yf, double thf, const std::vector<evacuation::Polygon> &obstacles, const evacuation::Polygon &map);

        Curve **multipointShortestPath(DubinsPoint **points, int numberOfPoints, const std::vector<evacuation::Polygon> &obstacles, const evacuation::Polygon &map);

        static std::vector<evacuation::Pose> interpolateCurves(Curve **curves, int size, int points_per_arc);

    private:
        /**
         * @brief The total possible curves to consider when solving a dubins problem
         */
        const int TOTAL_POSSIBLE_CURVES = 10;

        /**
         * @brief The types of curves to consider when solving a dubins problem
         */
        enum possible_curves {
            LSL,
            RSR,
            LSR,
            RSL,
            RLR,
            LRL,
            LS,
            RS,
            SL,
            SR
        };

        /**
         * @brief The curvature signs, where each element corresponds to one of the possible_curves
         */
        const int ksigns[6][3] = {
                {1,  0,  1},
                {-1, 0,  -1},
                {1,  0,  -1},
                {-1, 0,  1},
                {-1, 1,  -1},
                {1,  -1, 1},
        };

        /**
         * @brief All the possible angles handled by the solution of the multipoint dubins problem
         */
        const double multipointAngles[8] = {0, M_PI / 4, M_PI / 2, 3.0 / 4 * M_PI, M_PI, 5.0 / 4 * M_PI, 3.0 / 2 * M_PI, 7.0 / 4 * M_PI};

        /**
         * @brief Bound on maximum path curvature
         */
        double k_max = 1;

        /**
         * @brief Given a path of infinite points, what is the discritizer size? Expressed in meters
         */
        double discretizer_size = 0.005;

        /**
         * @brief Check the validity of a provided Dubins solution
         *
         * @param curve_segments
         * @param k0
         * @param k1
         * @param k2
         * @param th0
         * @param thf
         * @return true If the solution is valid
         * @return false If the solution is not valid
         */
        static bool areCurvesValid(CurveSegmentsResult *curve_segments_result, double k0, double k1, double k2, double th0, double thf);

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
        DubinsResult *scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf) const;

        /**
         * @brief Return to the initial scaling
         *
         * @param lambda
         * @param curve_segments
         * @return CurveSegmentsResult*
         */
        static CurveSegmentsResult *scaleFromStandard(double lambda, CurveSegmentsResult *curve_segments);

        // Functions to create a solution for the dubins problem, using different types of curves
        static CurveSegmentsResult *curveLSL(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveRSR(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveLSR(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveRSL(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveRLR(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveLRL(double scaled_th0, double scaled_thf, double scaled_k_max);

        static std::vector<evacuation::Pose> interpolateArc(Arc *arc, int num_points);
    };
}

#endif