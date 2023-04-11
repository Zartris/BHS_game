//
// Created by zartris on 4/10/23.
//

#ifndef BHS_GAME_INFEED_HPP
#define BHS_GAME_INFEED_HPP

# include "Agent.hpp"
#include "bhs_game/utils/config.h"

namespace bhs_game {
    class AInfeed : public Agent {
    private:
        // Add private members here
        InfeedParams infeed_config;
    public:
        AInfeed(int uniqueId, Config config) :
                Agent(uniqueId, "AInfeed", false, true),
                infeed_config(config.infeed_params) {
        }

        void step(double dt) override {};
    };
}


#endif //BHS_GAME_INFEED_HPP
