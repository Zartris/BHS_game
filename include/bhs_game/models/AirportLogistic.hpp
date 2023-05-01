//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_AIRPORTLOGISTIC_HPP
#define BHS_GAME_AIRPORTLOGISTIC_HPP

#include "Model.hpp"
#include "bhs_game/agents/AGV.hpp"
#include "bhs_game/agents/Server.hpp"
#include "bhs_game/utils/config.h"
#include <yaml-cpp/yaml.h>


namespace bhs_game {
    class AirportLogistic : public Model {
    private:
        Config config = Config();
        Eigen::MatrixXi obstacleMap;
        Eigen::MatrixXi chargingStationMap;
        Eigen::MatrixXi InfeedMap;
        Eigen::MatrixXi chuteMap;


    public:
        AirportLogistic(int seed = -1, bool allow_gpu = false);

        void populate_world();


        void populate_world_from_file(const std::string &file_name);

//        void populate_world_from_config(const std::string &config_file);

        void populate_world_from_image(const Config config);

//        void populate_world_from_json(const std::string &json_file);


    };
} // bhs_game


#endif //BHS_GAME_AIRPORTLOGISTIC_HPP
