//
// Created by zartris on 4/10/23.
//

#ifndef BHS_GAME_CHUTE_H
#define BHS_GAME_CHUTE_H

#include "Agent.hpp"
#include "bhs_game/utils/config.h"

namespace bhs_game {
    class AChute : public Agent {
    private:
        // Add private members here
        ChuteParams chute_config;
    public:
        AChute(int uniqueId, Config config) :
                Agent(uniqueId, "AChute", false, true),
                chute_config(config.chute_params) {
        }

        void step(double dt) override {};

    };
}


#endif //BHS_GAME_CHUTE_H
