//
// Created by nicola on 29/08/23.
//

#include <valarray>
#include "coordination.h"

using evacuation::Pose;
using std::vector;

namespace coordination {

    vector<vector<Pose>> getPathsWithoutRobotCollisions(vector<Pose> path1, vector<Pose> path2, vector<Pose> path3, double robotRadius) {
        bool path1_finished = false;
        bool path2_finished = false;
        bool path3_finished = false;
        int path1_index = 0;
        int path2_index = 0;
        int path3_index = 0;
        bool p1IntersectP2 = false;
        bool p1IntersectP3 = false;
        bool p2IntersectP3 = false;
        vector<Pose> safePath1;
        vector<Pose> safePath2;
        vector<Pose> safePath3;
        while (!path1_finished || !path2_finished || !path3_finished) {
            // Check if path1 in position path1_index intersects with path2 in position path2_index only if path1 is not finished
            if (!path1_finished) {
                Pose p1 = path1[path1_index];
                if (!path2_finished) {
                    Pose p2 = path2[path2_index];
                    if (!intersect(p1, p2, robotRadius)) {
                        // P1 does not intersect P2. Now, P3 must be checked to consider P1 safe
                        p1IntersectP2 = false;
                        if (!path3_finished) {
                            Pose p3 = path3[path3_index];
                            if (!intersect(p1, p3, robotRadius)) {
                                // P1 does not intersect P3. Now, P2 must be checked to consider P1 safe
                                p1IntersectP3 = false;
                            } else {
                                // P1 intersects P3.
                                p1IntersectP3 = true;
                            }
                        } else {
                            // P3 is finished. Now, P2 must be checked to consider P1 safe
                            p1IntersectP3 = false;
                        }
                    } else {
                        // P1 intersects P2.
                        p1IntersectP2 = true;
                    }
                } else {
                    // P2 is finished. Now, P3 must be checked to consider P1 safe
                    p1IntersectP2 = false;
                    if (!path3_finished) {
                        Pose p3 = path3[path3_index];
                        if (!intersect(p1, p3, robotRadius)) {
                            // P1 does not intersect P3.
                            p1IntersectP3 = false;
                        } else {
                            // P1 intersects P3.
                            p1IntersectP3 = true;
                        }
                    } else {
                        // P3 is finished. Now, P2 must be checked to consider P1 safe
                        p1IntersectP3 = false;
                    }
                }
            }

            if (!path2_finished) {
                Pose p2 = path2[path2_index];
                if (!path3_finished) {
                    Pose p3 = path3[path3_index];
                    if (!intersect(p2, p3, robotRadius)) {
                        // P2 does not intersect P3.
                        p2IntersectP3 = false;
                    } else {
                        // P2 intersects P3.
                        p2IntersectP3 = true;
                    }
                } else {
                    // P3 is finished.
                    p2IntersectP3 = false;
                }
            }

            if (!path1_finished) {
                safePath1.push_back(path1[path1_index]);
                path1_index++;
            } else {
                p1IntersectP2 = false;
                p1IntersectP3 = false;
            }
            if (!path2_finished) {
                if (p1IntersectP2) {
                    safePath2.push_back(path2[path2_index - 1]); // It raise exception only if the first points of the paths intersect. It means that they spawn directly with collision.
                } else {
                    safePath2.push_back(path2[path2_index]);
                    path2_index++;
                }
            } else {
                p2IntersectP3 = false;
            }
            if (!path3_finished) {
                if (p1IntersectP3) {
                    safePath3.push_back(path3[path3_index - 1]); // It raise exception only if the first points of the paths intersect. It means that they spawn directly with collision.
                } else if (p2IntersectP3) {
                    safePath3.push_back(path3[path3_index - 1]); // It raise exception only if the first points of the paths intersect. It means that they spawn directly with collision.
                } else {
                    safePath3.push_back(path3[path3_index]);
                    path3_index++;
                }
            }

            if (path1_index == int(path1.size())) {
                path1_finished = true;
            }
            if (path2_index == int(path2.size())) {
                path2_finished = true;
            }
            if (path3_index == int(path3.size())) {
                path3_finished = true;
            }
        }
        vector<vector<Pose>> safePaths;
        safePaths.push_back(safePath1);
        safePaths.push_back(safePath2);
        safePaths.push_back(safePath3);
        return safePaths;
    }

    bool intersect(Pose p1, Pose p2, double robotRadius) {
        // (R0 - R1)^2 <= (x0 - x1)^2 + (y0 - y1)^2 <= (R0 + R1)^2
        double val = std::pow((p1.position.x - p2.position.x), 2) + std::pow((p1.position.y - p2.position.y), 2);
        return val >= 0 && val <= std::pow((robotRadius * 2), 2);
    }

}
