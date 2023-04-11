//
// Created by zartris on 4/10/23.
//

#ifndef BHS_GAME_OBSTACLE_HPP
#define BHS_GAME_OBSTACLE_HPP

#include "Agent.hpp"
#include "bhs_game/utils/config.h"

namespace bhs_game {
    class AObstacle : public Agent {
    private:
        // Add private members here
        ObstacleParams obstacle_config;
    public:
        AObstacle(int uniqueId, const Config& config) :
                Agent(uniqueId, "AObstacle", true, true),
                obstacle_config(config.obstacle_params) {

        }

        void step(double dt) override {};


    };
}


#endif //BHS_GAME_OBSTACLE_HPP
