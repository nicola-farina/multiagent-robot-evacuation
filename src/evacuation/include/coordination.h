//
// Created by nicola on 29/08/23.
//

#ifndef EVACUATION_COORDINATION_H
#define EVACUATION_COORDINATION_H

#include "environment.hpp"

namespace coordination {
    std::vector<std::vector<evacuation::Pose>> getPathsWithoutRobotCollisions(std::vector<evacuation::Pose> path1, std::vector<evacuation::Pose> path2, std::vector<evacuation::Pose> path3);

    bool intersect(evacuation::Pose p1, evacuation::Pose p2, double robotRadius);
}

#endif
