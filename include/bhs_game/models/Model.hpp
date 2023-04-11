//
// Created by zartris on 4/8/23.
//
#pragma once

#include <random>
#include "bhs_game/Scheduler.hpp"
#include "bhs_game/World.hpp"

#ifndef BHS_VIS_MODEL_H
#define BHS_VIS_MODEL_H

namespace bhs_game {

    class Model {
        // Variables
    private:
        bool running;
        // Replace schedule with the appropriate type for your project
        // schedule_object_type* schedule;
        int current_id;
        int _seed;
        float physic_step_time = 0.1f;
        int step_count = 0;
    protected:
        BaseScheduler *scheduler = nullptr;
        ContinuousSpace *world = nullptr;
        std::mt19937 random;
    public:
        explicit Model(int seed = -1);

        void run(int epochs);

        void play_until_stopped();

        virtual void step();

        int next_id();

        void reset_randomizer(int seed = -1);

        void set_physic_step_time(float time) {
            physic_step_time = time;
        }

        [[nodiscard]] float get_physic_step_time() const {
            return physic_step_time;
        }


    };

} // bhs_game

#endif //BHS_VIS_MODEL_H
