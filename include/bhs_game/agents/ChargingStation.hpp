//
// Created by zartris on 4/10/23.
//

#ifndef BHS_GAME_CHARGINGSTATION_HPP
#define BHS_GAME_CHARGINGSTATION_HPP

# include "Agent.hpp"
# include "bhs_game/utils/config.h"

namespace bhs_game {
    class AChargingStation : public Agent {
    private:
        ChargingStationParams cs_config;
    public:
        AChargingStation(int uniqueId, const Config& config) :
                Agent(uniqueId, "AChargingStation", false, true),
                cs_config(config.charging_station_params) {
        }

        void step(double dt) override {};
    };
}

#endif //BHS_GAME_CHARGINGSTATION_HPP
