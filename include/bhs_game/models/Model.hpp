//
// Created by zartris on 4/8/23.
//
#pragma once

#include <random>

#ifndef BHS_VIS_MODEL_H
#define BHS_VIS_MODEL_H

namespace bhs_game {

    class Model {
    public:
        explicit Model(int seed = -1);

        void run_model();

        virtual void step();

        int next_id();

        void reset_randomizer(int seed = -1);

        void set_physic_step_time(float time) {
            physic_step_time = time;
        }

        [[nodiscard]] float get_physic_step_time() const {
            return physic_step_time;
        }


    private:
        bool running;
        // Replace schedule with the appropriate type for your project
        // schedule_object_type* schedule;
        int current_id;
        int _seed;
        std::mt19937 random;
        float physic_step_time = 0.1f;

    };

} // bhs_game

#endif //BHS_VIS_MODEL_H
