#ifndef BHS_VIS_WORLD_H
#define BHS_VIS_WORLD_H

#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>
#include "bhs_game/agents/Agent.hpp"
#include "bhs_game/utils/tensor_alias.h"

namespace bhs_game {
    class ContinuousSpace {
    public:
        ContinuousSpace(TScalarDouble x_max, TScalarDouble y_max, bool torus,
                        TScalarDouble x_min = torch::scalar_tensor(0, TOptions(torch::kInt, device)),
                        TScalarDouble y_min = torch::scalar_tensor(0, TOptions(torch::kInt, device)));

        void placeAgent(Agent *agent, TensorXDouble pos);

        void moveAgent(Agent *agent, TensorXDouble &pos);

        void removeAgent(Agent *agent);

        // Additional methods go here

    private:
        TScalarDouble x_min;
        TScalarDouble x_max;
        TScalarDouble width;
        TScalarDouble y_min;
        TScalarDouble y_max;
        TScalarDouble height;
        bool torus;
        Tensor2Double center;
        Tensor2Double size;

        std::vector<Tensor3Double> agent_points;
        std::map<size_t, Agent *> index_to_agent;
        std::map<Agent *, size_t> agent_to_index;
        std::vector<int> static_agents;
        std::vector<int> dynamic_agents;
        std::vector<int> collision_agents;


        // Additional private methods go here
        void invalidate_agent_cache();

        TensorXDouble torus_adj(TensorXDouble &pos);

        bool out_of_bounds(const TensorXDouble &pos) const;

        void buildAgentCache();

        std::vector<Agent *> getNeighborhoodFromList(const TensorXDouble &pos, double radius,
                                                     std::vector<int> &agent_index_search_list,
                                                     bool include_center = false);


    };
} // bhs_game

#endif //BHS_VIS_WORLD_H
