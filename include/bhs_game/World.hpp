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

        void placeAgent(Agent *agent, const Eigen::Vector3d &pos);

        void moveAgent(Agent *agent, const Eigen::Vector3d &pos);

        void removeAgent(Agent *agent);

        // Additional methods go here

    private:
        double x_min;
        double x_max;
        double width;
        double y_min;
        double y_max;
        double height;
        bool torus;
        Eigen::Array2d center;
        Eigen::Array2d size;

        std::vector<Eigen::Vector3d> agent_points;
        std::map<size_t, Agent *> index_to_agent;
        std::map<Agent *, size_t> agent_to_index;
        std::vector<int> static_agents;
        std::vector<int> dynamic_agents;
        std::vector<int> collision_agents;

        // Additional private methods go here
        void invalidate_agent_cache();

        Eigen::Vector3d torus_adj(const Eigen::Vector3d &pos);

        bool out_of_bounds(const Eigen::Vector3d &pos) const;

        void buildAgentCache();

        std::vector<Agent *> getNeighborhoodFromList(const Eigen::Vector3d &pos, double radius,
                                                     std::vector<int> &agent_index_search_list, bool include_center = false);


    };
} // bhs_game

#endif //BHS_VIS_WORLD_H
