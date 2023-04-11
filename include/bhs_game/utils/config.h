//
// Created by zartris on 4/10/23.
//

#ifndef BHS_GAME_CONFIG_H
#define BHS_GAME_CONFIG_H

#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"


struct RandomScenario {
    int num_obstacles;
    int num_chutes;
    int num_infeed;
    int num_charging_stations;
    int num_agvs;
};

struct ImageScenario {
    std::string img_dir_path;
    std::string agv_img_name;
    std::vector<int> agv_color;
    std::string chute_img_name;
    std::vector<int> chute_color;
    std::string infeed_img_name;
    std::vector<int> infeed_color;
    std::string obstacle_img_name;
    std::vector<int> obstacle_color;
    std::string charging_station_img_name;
    std::vector<int> charging_station_color;
    std::vector<int> outside_color;
    std::vector<int> floor_color;
    int num_obstacles;
    int num_chutes;
    int num_infeed;
    int num_charging_stations;
    int num_agvs;
};

struct AGVParams {
    double max_battery;
    double move_cost;
    double wait_cost;
    double idle_cost;
    double radius;
    double max_speed;
    double max_acceleration;
    double max_deceleration;
    double wheel_radius;
    double wheel_distance;
};

struct ChargingStationParams {
    double charge_amount;
    double radius;
};

struct ChuteParams {
    double radius;
};

struct InfeedParams {
    double radius;
};

struct ObstacleParams {
    double radius;
};

struct PathPlanner {
    int max_comp_time;
    int max_timestep;
};

struct Canvas {
    bool live_visualisation;
    int battery_color_interval;
    int width;
    int height;
    double scale;
};

class Config {
public:
    int seed;
    int port;
    int max_steps;
    bool performance_test;
    bool verbose;
    int grid_width;
    int grid_height;
    bool continuous;
    double dt;
    double pixel_size_in_meters;
    std::string scenario_type;

    RandomScenario random_scenario;
    ImageScenario image_scenario;

    AGVParams agv_params;
    ChargingStationParams charging_station_params;
    ChuteParams chute_params;
    InfeedParams infeed_params;
    ObstacleParams obstacle_params;

    PathPlanner path_planner;
    Canvas canvas;

    void load_from_yaml(const std::string &file_path);
};


#endif //BHS_GAME_CONFIG_H
