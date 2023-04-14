//
// Created by zartris on 4/8/23.
//

#include "bhs_game/models/Model.hpp"

namespace bhs_game {
    Model::Model(int seed, bool allow_gpu) : running(true), current_id(0), _seed(seed) {
        if (_seed == -1) {
            std::random_device rd;
            _seed = rd();
        }
        random.seed(_seed);

        if ( allow_gpu && torch::cuda::is_available()) {
            std::cout << "CUDA is available! Training on GPU." << std::endl;
            device = torch::kCUDA;
        }
    }

    /*
     * This is the main loop of the model. It is called every time step.
     * It is called by the run function.
     * It runs through a few steps:
     * 1. It calls the scheduler to step through the agents and update behaviors
     * 2. It calls the world to step through the agents and update positions and resolve collisions
     *
     */
    void Model::step() {
        // Implement the single step logic here
        printf("Step %i \n", step_count);
        scheduler->step(physic_step_time);
//        world->step(physic_step_time);
        step_count += 1;
    }

    int Model::next_id() {
        current_id += 1;
        return current_id;
    }

    void Model::reset_randomizer(int seed) {
        if (seed == -1) {
            seed = _seed;
        }
        random.seed(seed);
        _seed = seed;
    }

    void Model::play_until_stopped() {
        running = true;
        while (running) {
            step();
        }
    }

    void Model::run(int epochs) {
        running = true;
        for (int i = 0; i < epochs; i++) {
            printf("\nEpoch %i \n", i);
            step();
        }
        running = false;
    }

} // bhs_game