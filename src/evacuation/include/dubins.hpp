//
// Created by luca on 14/08/23.
//

#ifndef DUBINS_HPP
#define DUBINS_HPP

#include "dubins_utils.hpp"
#include "environment.hpp"
#include "coordination.hpp"
#include <vector>
#include <valarray>


namespace dubins {

    struct Line {
        double x, y, th;

        Line(double s, double x0, double y0, double th0, double k) {
            x = x0 + (s * sinc(k * s / 2.0) * cos(th0 + ((k * s) / 2)));
            y = y0 + (s * sinc(k * s / 2.0) * sin(th0 + ((k * s) / 2)));
            th = mod2pi(th0 + (k * s));
        }
    };

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

    struct DubinsResult {
        double scaled_th0, scaled_thf, scaled_k_max, lambda;

        DubinsResult(double scaled_th0, double scaled_thf, double scaled_k_max, double lambda) : scaled_th0(scaled_th0), scaled_thf(scaled_thf), scaled_k_max(scaled_k_max), lambda(lambda) {}
    };

    struct CurveSegmentsResult {
        bool ok;
        double s1, s2, s3;

        CurveSegmentsResult(bool ok, double s1, double s2, double s3) : ok(ok), s1(s1), s2(s2), s3(s3) {}
    };

    class Dubins {
    public:

        Dubins() = default;

        explicit Dubins(double k_max, double discretizer_size);

        Curve *findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf);

        static bool arcLineIntersection(Arc *arc, DubinsPoint point1, DubinsPoint point2, std::vector<DubinsPoint> &pts, std::vector<double> &t);

        static bool lineLineIntersection(DubinsPoint p1, DubinsPoint p2, DubinsPoint p3, DubinsPoint p4, std::vector<DubinsPoint> &pts, std::vector<double> &ts);

        double *getAnglesShortestPath(DubinsPoint **points, int numberOfPoints, const std::vector<evacuation::Polygon> &obstacles, const evacuation::Polygon &map);

        static std::vector<std::vector<evacuation::Point>> getObstaclesAndMapLines(std::vector<evacuation::Polygon> obstacles, evacuation::Polygon map);

        Curve *ShortestPathWithoutCollisions(double x0, double y0, double th0, double xf, double yf, double thf, const std::vector<evacuation::Polygon> &obstacles, const evacuation::Polygon &map);

        Curve **multipointShortestPath(DubinsPoint **points, int numberOfPoints, const std::vector<evacuation::Polygon> &obstacles, const evacuation::Polygon &map);

        static std::vector<coordination::PoseForCoordination> interpolateCurves(Curve **curves, int size, int points_per_arc);

    private:

        const int TOTAL_POSSIBLE_CURVES = 10;

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

        const int ksigns[6][3] = {
                {1,  0,  1},
                {-1, 0,  -1},
                {1,  0,  -1},
                {-1, 0,  1},
                {-1, 1,  -1},
                {1,  -1, 1},
        };

        const double multipointAngles[8] = {0, M_PI / 4, M_PI / 2, 3.0 / 4 * M_PI, M_PI, 5.0 / 4 * M_PI, 3.0 / 2 * M_PI, 7.0 / 4 * M_PI};

        double k_max = 1;

        double discretizer_size = 0.005;

        static bool areCurvesValid(CurveSegmentsResult *curve_segments_result, double k0, double k1, double k2, double th0, double thf);

        DubinsResult *scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf) const;

        static CurveSegmentsResult *scaleFromStandard(double lambda, CurveSegmentsResult *curve_segments);

        static CurveSegmentsResult *curveLSL(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveRSR(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveLSR(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveRSL(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveRLR(double scaled_th0, double scaled_thf, double scaled_k_max);

        static CurveSegmentsResult *curveLRL(double scaled_th0, double scaled_thf, double scaled_k_max);

        static std::vector<coordination::PoseForCoordination> interpolateArc(Arc *arc, int num_points, double timeFromInitial);
    };
}

#endif