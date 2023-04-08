//
// Created by zartris on 4/8/23.
//

#ifndef BHS_VIS_WORLD_H
#define BHS_VIS_WORLD_H

#include <Eigen/Core>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>
#include "bhs_game/agents/Agent.hpp"

namespace bhs_game {
    class ContinuousSpace {
    public:
        ContinuousSpace(double x_max, double y_max, bool torus, double x_min = 0, double y_min = 0);

        void place_agent(Agent *agent, const Eigen::Vector2d &pos);

        void move_agent(Agent *agent, const Eigen::Vector2d &pos);

        void remove_agent(Agent *agent);

        // Additional methods go here

    private:
        double x_min;
        double x_max;
        double width;
        double y_min;
        double y_max;
        double height;
        bool torus;
        Eigen::Vector2d center;
        Eigen::Vector2d size;

        std::vector<Eigen::Vector2d> agent_points;
        std::map<size_t, Agent *> index_to_agent;
        std::map<Agent *, size_t> agent_to_index;

        // Additional private methods go here
        void invalidate_agent_cache();

        Eigen::Vector2d torus_adj(const Eigen::Vector2d &pos);

        bool out_of_bounds(const Eigen::Vector2d &pos) const;

        void build_agent_cache();


    };
} // bhs_game

#endif //BHS_VIS_WORLD_H
