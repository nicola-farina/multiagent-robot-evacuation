//
// Created by luca on 14/08/23.
//

#include "models.hpp"


int main(int argc, char** argv) {
    Polygon map = Polygon({Point(0, 0), Point(0, 10), Point(10, 10), Point(10, 0)});
    Polygon obstacle = Polygon({Point(2, 2), Point(2, 4), Point(4, 4), Point(4, 2)});
    std::vector<Polygon> obstacles = {obstacle};
    std::vector<Robot> robots = {Robot(Polygon({Point(0, 0), Point(0, 1), Point(1, 1), Point(1, 0)}), 0.5)};
    std::vector<Polygon> gates = {Polygon({Point(9, 9), Point(9, 10), Point(10, 10), Point(10, 9)})};
    environment::Environment env(map, obstacles, robots, gates);
    std::cout << "Map: " << std::endl;
}
