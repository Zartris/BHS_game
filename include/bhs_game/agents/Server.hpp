//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_SERVER_H
#define BHS_GAME_SERVER_H

#include "Agent.hpp"
#include "bhs_game/utils/global_device.h"
#include "bhs_game/utils/tensor_alias.h"

namespace bhs_game {
    class AServer : public Agent {
    private:
        // Add private members here
        // 1. list of agents that are connected to this server
        // 2. Event callback for when agent sends requests
        // 3. time when we can release the results from requests
        long time_to_release_results = 0;
        double time_of_request = 0.0;
        bool request_received = false;
        bool result_pending = false;

    public:

        explicit AServer(int uniqueId) : Agent(uniqueId, "AServer", false, true) {

        }

        ~AServer() override = default;

        void step(double dt) override;

    private:
        // Add private methods here
        std::vector<TensorXDouble> computePath();


    public:
        // getter and setters
        void setRequestReceived(bool requestReceived) {
            request_received = requestReceived;
        }

    };

} // bhs_game





#endif //BHS_GAME_SERVER_H
