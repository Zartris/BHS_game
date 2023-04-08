//
// Created by zartris on 4/8/23.
//

#include <chrono>
#include "bhs_game/agents/Server.hpp"

namespace bhs_game {
    void AServer::step(double dt) {
        // Implement the single step logic here
        // 1. Check if there are any requests from agents
        if (request_received) {
            // new information has arrived, we need to recompute
            auto start_time = std::chrono::high_resolution_clock::now();
            computePath();
            auto end_time = std::chrono::high_resolution_clock::now();
            // todo:: Might want to round this up or down to closes dt divisible number or just have the offset be com delay
            time_to_release_results = std::chrono::duration_cast<std::chrono::microseconds>(
                    end_time - start_time).count();
            result_pending = true;
        }
        // 2. If there are requests, check if we can release the results
        if (result_pending) {
            time_to_release_results -= static_cast<long>(dt * 1000);
            // 3. If we can release the results, release the results
            if (time_to_release_results <= 0) {
                // we can release the results
                result_pending = false;
            }
        }
        // 4. If we can't release the results, do nothing
    }

    std::vector<Eigen::Vector2d> AServer::computePath() {
        return std::vector<Eigen::Vector2d>();
    }
} // bhs_game
