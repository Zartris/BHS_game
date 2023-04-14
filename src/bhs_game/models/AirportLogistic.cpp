//
// Created by zartris on 4/8/23.
//


#include <filesystem>
#include <iostream>
#include "bhs_game/models/AirportLogistic.hpp"
#include "bhs_game/utils/image_utils.hpp"
#include "bhs_game/agents/Obstacle.hpp"
#include "bhs_game/agents/ChargingStation.hpp"
#include "bhs_game/agents/Infeed.hpp"
#include "bhs_game/agents/Chute.hpp"

namespace bhs_game {
    AirportLogistic::AirportLogistic(int seed, bool allow_gpu) : Model(seed, allow_gpu) {
        scheduler = new TypeOrderedScheduler(true, random);
        populate_world_from_file("/home/zartris/code/cpp/BHS_vis/third_party/BHS_game/configs/config.yaml");
        dynamic_cast<TypeOrderedScheduler *>(scheduler)->setStepOrder({
                                                                              "AServer",
                                                                              "AAGV",
                                                                      });
    }

    void AirportLogistic::populate_world() {
        // Implement the world population logic here
        for (int i = 0; i < 10; i++) {
            auto agent = new AAGV(next_id(),
                                  {100., 100.},
                                  MotorConfig(10., 1., 1., 0.1),
                                  0.1, 0.5);
            scheduler->addAgent(agent);
        }

        for (int i = 0; i < 1; i++) {
            auto server = new AServer(next_id());
            scheduler->addAgent(server);
        }
    }

    void AirportLogistic::populate_world_from_file(const std::string &file_name) {
        // check format of file
        if (file_name.substr(file_name.find_last_of(".") + 1) == "yaml") {
            config.load_from_yaml(file_name);
            // Todo:: change to read from other file
            world = new ContinuousSpace(
                    torch::scalar_tensor(config.grid_width, TOptions(torch::kDouble, device)),
                    torch::scalar_tensor(config.grid_height, TOptions(torch::kDouble, device)),
                    false);
            populate_world_from_image(config);
            int debug = 0;
        } else {
            std::cerr << "File format not supported" << std::endl;
        }
    }

    void AirportLogistic::populate_world_from_image(const Config config) {
        std::cout << "Populating world from image" << std::endl;
        ImageScenario scenarioConfig = config.image_scenario;
        torch::Tensor emptySpaceTensor = torch::ones({config.grid_width, config.grid_height});
        obstacleMap = torch::zeros({config.grid_width, config.grid_height});
        chargingStationMap = torch::zeros({config.grid_width, config.grid_height});
        InfeedMap = torch::zeros({config.grid_width, config.grid_height});
        chuteMap = torch::zeros({config.grid_width, config.grid_height});


        // 1. First check for images that exist and insert in map
        /////////////////////// OBSTACLES FROM IMAGE /////////////////////////////
        std::filesystem::path obstacleImgPath =
                std::filesystem::path(scenarioConfig.img_dir_path) / scenarioConfig.obstacle_img_name;

        if (std::filesystem::exists(obstacleImgPath)) {
            // Code to process obstacles from image
            std::cout << "Inserting obstacles into map" << std::endl;
            ImageVector obstacleImg = ImageUtils::loadImageToVector(obstacleImgPath.string());
            auto rows = obstacleImg.size();
            auto cols = obstacleImg[0].size();
            if (rows != config.grid_height || cols != config.grid_width) {
                std::cerr << "Obstacle map does not have the same size as the grid." << std::endl;
                // ... Add the rest of the error message and handle the error as needed.
            }
            // Process the obstacleImg and update the emptySpaceMap, obstacleMap, and grid
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    auto val = obstacleImg[i][j];
                    if (val == scenarioConfig.obstacle_color) { // Assuming obstacles have a non-zero value in the image
                        AObstacle *obstacle = new AObstacle(next_id(), config);
                        world->placeAgent(obstacle, torch::tensor({static_cast<double>(i), static_cast<double>(j), 0.},
                                                                  TOptions(torch::kDouble, device)));

                        emptySpaceTensor.index_put_({i, j}, 0); // Mark the cell as occupied in the emptySpaceMap
//                        obstacleMap(i, j) = 1;
                    }
                }
            }
        }
        /////////////////////// CHARGING STATIONS FROM IMAGE /////////////////////////////
        std::filesystem::path chargingStationImgPath =
                std::filesystem::path(scenarioConfig.img_dir_path) / scenarioConfig.charging_station_img_name;

        if (std::filesystem::exists(chargingStationImgPath)) {
            // Code to process charging stations from image
            std::cout << "Inserting charging stations into map" << std::endl;
            ImageVector chargingStationImg = ImageUtils::loadImageToVector(chargingStationImgPath.string());
            auto rows = chargingStationImg.size();
            auto cols = chargingStationImg[0].size();
            if (rows != config.grid_height || cols != config.grid_width) {
                std::cerr << "Charging station map does not have the same size as the grid." << std::endl;
                // ... Add the rest of the error message and handle the error as needed.
            }
            // Process the chargingStationImg and update the emptySpaceMap, chargingStationMap, and grid
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    auto val = chargingStationImg[i][j];
                    if (val ==
                        scenarioConfig.charging_station_color) { // Assuming charging stations have a non-zero value in the image
                        AChargingStation *chargingStation = new AChargingStation(next_id(), config);
                        world->placeAgent(chargingStation,
                                          torch::tensor({static_cast<double>(i), static_cast<double>(j), 0.},
                                                        TOptions(torch::kDouble, device)));
                        scheduler->addAgent(chargingStation);
                        emptySpaceTensor.index_put_({i, j}, 0); // Mark the cell as occupied in the emptySpaceMap
//                        chargingStationMap(i, j) = 1;
                    }
                }
                std::cout << std::endl;
            }
        }

        /////////////////////// INFEED FROM IMAGE /////////////////////////////
        std::filesystem::path infeedImgPath =
                std::filesystem::path(scenarioConfig.img_dir_path) / scenarioConfig.infeed_img_name;

        if (std::filesystem::exists(infeedImgPath)) {
            // Code to process infeed from image
            std::cout << "Inserting infeed into map" << std::endl;
            ImageVector infeedImg = ImageUtils::loadImageToVector(infeedImgPath.string());
            auto rows = infeedImg.size();
            auto cols = infeedImg[0].size();
            if (rows != config.grid_height || cols != config.grid_width) {
                std::cerr << "Infeed map does not have the same size as the grid." << std::endl;
                // ... Add the rest of the error message and handle the error as needed.
            }
            // Process the infeedImg and update the emptySpaceMap, chargingStationMap, and grid
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    auto val = infeedImg[i][j];
                    if (val ==
                        scenarioConfig.infeed_color) { // Assuming charging stations have a non-zero value in the image
                        AInfeed *infeed = new AInfeed(next_id(), config);
                        world->placeAgent(infeed, torch::tensor({static_cast<double>(i), static_cast<double>(j), 0.},
                                                                TOptions(torch::kDouble, device)));
                        scheduler->addAgent(infeed);
                        emptySpaceTensor.index_put_({i, j}, 0); // Mark the cell as occupied in the emptySpaceMap

//                        InfeedMap(i, j) = 1;
                    }
                }
            }
        }

        /////////////////////// CHUTE FROM IMAGE /////////////////////////////
        std::filesystem::path chuteImgPath =
                std::filesystem::path(scenarioConfig.img_dir_path) / scenarioConfig.chute_img_name;

        if (std::filesystem::exists(chuteImgPath)) {
            std::cout << "Inserting chute into map" << std::endl;
            ImageVector chuteImg = ImageUtils::loadImageToVector(chuteImgPath.string());
            auto rows = chuteImg.size();
            auto cols = chuteImg[0].size();

            if (rows != config.grid_height || cols != config.grid_width) {
                std::cerr << "Chute map does not have the same size as the grid." << std::endl;
                // ... Add the rest of the error message and handle the error as needed.
            }
            // Process the chuteImg and update the emptySpaceMap, chargingStationMap, and grid
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    auto val = chuteImg[i][j];
                    if (val ==
                        scenarioConfig.chute_color) { // Assuming charging stations have a non-zero value in the image
                        AChute *chute = new AChute(next_id(), config);
                        world->placeAgent(chute, torch::tensor({static_cast<double>(i), static_cast<double>(j), 0.},
                                                               TOptions(torch::kDouble, device)));
                        scheduler->addAgent(chute);
                        emptySpaceTensor.index_put_({i, j}, 0); // Mark the cell as occupied in the emptySpaceMap

//                        chuteMap(i, j) = 1;
                    }
                }
            }
        }

        /////////////////////// AGV's FROM IMAGE /////////////////////////////
        std::filesystem::path agvImgPath =
                std::filesystem::path(scenarioConfig.img_dir_path) / scenarioConfig.agv_img_name;
        int agvCount = 0;
        if (std::filesystem::exists(agvImgPath)) {
            std::cout << "Inserting agv's into map" << std::endl;
            ImageVector agvImg = ImageUtils::loadImageToVector(agvImgPath.string());
            auto rows = agvImg.size();
            auto cols = agvImg[0].size();

            if (rows != config.grid_height || cols != config.grid_width) {
                std::cerr << "AGV map does not have the same size as the grid." << std::endl;
                // ... Add the rest of the error message and handle the error as needed.
            }
            // Process the agvImg and update the emptySpaceMap, chargingStationMap, and grid
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    auto val = agvImg[i][j];
                    if (val ==
                        scenarioConfig.agv_color) { // Assuming charging stations have a non-zero value in the image
                        AAGV *agv = new AAGV(next_id(), config);
                        world->placeAgent(agv, torch::tensor({static_cast<double>(i), static_cast<double>(j), 0.},
                                                             TOptions(torch::kDouble, device)));
                        scheduler->addAgent(agv);
                        emptySpaceTensor.index_put_({i, j}, 0); // Mark the cell as occupied in the emptySpaceMap

                        agvCount++;
                    }
                }
            }
        }

        /////////////////////// MAKE ONE SERVER /////////////////////////////
        AServer *server = new AServer(next_id());
        scheduler->addAgent(server);

        /////////////////////// MAKE REST OF AGV ////////////////////////////
        // Where returns [tensor, tensor] where tensor[0] is the row indices and tensor[1] is the col indices
        std::vector <torch::Tensor> free_space_indx = torch::where(emptySpaceTensor == 1);
        Tensor2Double t_free_space_indx = torch::stack({free_space_indx[0], free_space_indx[1]}, 0).t();

        std::vector<int> indices(t_free_space_indx.sizes()[0]);
        std::iota(indices.begin(), indices.end(), 0); // Fill with 0, 1, ..., size.
        std::shuffle(indices.begin(), indices.end(), random);
        int count = 0;
        for (int i = agvCount; i < scenarioConfig.num_agvs; ++i) {
            int random_indx = indices[count];
            int x = free_space_indx[0].index({random_indx}).item<int>();
            int y = free_space_indx[1].index({random_indx}).item<int>();
            AAGV *agv = new AAGV(next_id(), config);
            world->placeAgent(agv, torch::tensor({static_cast<double>(x), static_cast<double>(y), 0.},
                                                 TOptions(torch::kDouble, device)));
            scheduler->addAgent(agv);
        }
    }
}