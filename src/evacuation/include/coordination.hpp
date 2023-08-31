//
// Created by nicola on 29/08/23.
//

#ifndef EVACUATION_COORDINATION_HPP
#define EVACUATION_COORDINATION_HPP

#include "environment.hpp"

namespace coordination {

    struct PoseForCoordination {
        evacuation::Pose pose;
        double distanceFromInitial;

        PoseForCoordination();

        PoseForCoordination(double x, double y, double th, double distanceFromInitial);

        PoseForCoordination(evacuation::Pose pose, double distanceFromInitial);

        evacuation::Pose getPose() const;
    };

    struct RobotCoordination {
        std::vector<evacuation::Pose> path;
        double timeToWait;

        RobotCoordination(std::vector<evacuation::Pose> path, double timeToWait);
    };

    std::vector<RobotCoordination> getPathsWithoutRobotCollisions(std::vector<PoseForCoordination> path1, std::vector<PoseForCoordination> path2, std::vector<PoseForCoordination> path3, double robotRadius);

    bool intersect(evacuation::Pose p1, evacuation::Pose p2, double robotRadius);

    bool intersectAtSameTime(PoseForCoordination p1, PoseForCoordination p2, double robot1WaitTime, double robot2WaitTime);
}

#endif
