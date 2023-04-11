//
// Created by zartris on 4/10/23.
//

#include "bhs_game/utils/config.h"

void Config::load_from_yaml(const std::string &file_path) {
    YAML::Node config = YAML::LoadFile(file_path);

    seed = config["seed"].as<int>();
    port = config["port"].as<int>();
    max_steps = config["max_steps"].as<int>();
    performance_test = config["performance_test"].as<bool>();
    verbose = config["verbose"].as<bool>();
    grid_width = config["grid_width"].as<int>();
    grid_height = config["grid_height"].as<int>();
    continuous = config["continuous"].as<bool>();
    dt = config["dt"].as<double>();
    pixel_size_in_meters = config["pixel_size_in_meters"].as<double>();
    scenario_type = config["scenario_type"].as<std::string>();

    // Load random_scenario
    random_scenario.num_obstacles = config["random_scenario"]["num_obstacles"].as<int>();
    random_scenario.num_chutes = config["random_scenario"]["num_chutes"].as<int>();
    random_scenario.num_infeed = config["random_scenario"]["num_infeed"].as<int>();
    random_scenario.num_charging_stations = config["random_scenario"]["num_charging_stations"].as<int>();
    random_scenario.num_agvs = config["random_scenario"]["num_agvs"].as<int>();

    // Load image_scenario
    image_scenario.img_dir_path = config["image_scenario"]["img_dir_path"].as<std::string>();
    image_scenario.agv_img_name = config["image_scenario"]["agv_img_name"].as<std::string>();
    image_scenario.agv_color = config["image_scenario"]["agv_color"].as<std::vector<int>>();
    image_scenario.chute_img_name = config["image_scenario"]["chute_img_name"].as<std::string>();
    image_scenario.chute_color = config["image_scenario"]["chute_color"].as<std::vector<int>>();
    image_scenario.infeed_img_name = config["image_scenario"]["infeed_img_name"].as<std::string>();
    image_scenario.infeed_color = config["image_scenario"]["infeed_color"].as<std::vector<int>>();
    image_scenario.obstacle_img_name = config["image_scenario"]["obstacle_img_name"].as<std::string>();
    image_scenario.obstacle_color = config["image_scenario"]["obstacle_color"].as<std::vector<int>>();
    image_scenario.outside_color = config["image_scenario"]["outside_color"].as<std::vector<int>>();
    image_scenario.floor_color = config["image_scenario"]["floor_color"].as<std::vector<int>>();
    image_scenario.charging_station_img_name = config["image_scenario"]["charging_station_img_name"].as<std::string>();
    image_scenario.charging_station_color = config["image_scenario"]["charging_station_color"].as<std::vector<int>>();
    image_scenario.num_obstacles = config["image_scenario"]["num_obstacles"].as<int>();
    image_scenario.num_chutes = config["image_scenario"]["num_chutes"].as<int>();
    image_scenario.num_infeed = config["image_scenario"]["num_infeed"].as<int>();
    image_scenario.num_charging_stations = config["image_scenario"]["num_charging_stations"].as<int>();
    image_scenario.num_agvs = config["image_scenario"]["num_agvs"].as<int>();



    // Load agv_params
    agv_params.max_battery = config["agv_params"]["max_battery"].as<int>();
    agv_params.move_cost = config["agv_params"]["move_cost"].as<double>();
    agv_params.wait_cost = config["agv_params"]["wait_cost"].as<double>();
    agv_params.idle_cost = config["agv_params"]["idle_cost"].as<double>();
    agv_params.radius = config["agv_params"]["radius"].as<double>();
    agv_params.max_speed = config["agv_params"]["max_speed"].as<double>();
    agv_params.max_acceleration = config["agv_params"]["max_acceleration"].as<double>();
    agv_params.max_deceleration = config["agv_params"]["max_deceleration"].as<double>();
    agv_params.wheel_radius = config["agv_params"]["wheel_radius"].as<double>();
    agv_params.wheel_distance = config["agv_params"]["wheel_distance"].as<double>();

    // Load charging_station_params
    charging_station_params.charge_amount = config["charging_station_params"]["charge_amount"].as<double>();
    charging_station_params.radius = config["charging_station_params"]["radius"].as<double>();

    // Load chute_params
    chute_params.radius = config["chute_params"]["radius"].as<double>();

    // Load infeed_params
    infeed_params.radius = config["infeed_params"]["radius"].as<double>();

    // Load obstacle_params
    obstacle_params.radius = config["obstacle_params"]["radius"].as<double>();

    // Load path_planner
    path_planner.max_comp_time = config["path_planner"]["max_comp_time"].as<int>();
    path_planner.max_timestep = config["path_planner"]["max_timestep"].as<int>();

    // Load canvas
    canvas.live_visualisation = config["canvas"]["live_visualisation"].as<bool>();
    canvas.battery_color_interval = config["canvas"]["battery_color_interval"].as<int>();
    canvas.width = config["canvas"]["width"].as<int>();
    canvas.height = config["canvas"]["height"].as<int>();
    canvas.scale = config["canvas"]["scale"].as<double>();
}


