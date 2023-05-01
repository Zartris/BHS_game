#pragma once
#define GLM_FORCE_SWIZZLE

#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>
#include "bhs_game/agents/Agent.hpp"
#include "glm/gtx/norm.hpp"


namespace bhs_game {
    class ContinuousSpace {
    public:
        ContinuousSpace(double x_max, double y_max, bool torus,
                        double x_min = 0.,
                        double y_min = 0.);

        void placeAgent(Agent *agent, glm::dvec3 pos);

        void moveAgent(Agent *agent, glm::dvec3 &pos);

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
        glm::dvec2 center;
        glm::dvec2 size;

        std::vector<glm::dvec3> agent_points;
        std::map<size_t, Agent *> index_to_agent;
        std::map<Agent *, size_t> agent_to_index;
        std::vector<int> static_agents;
        std::vector<int> dynamic_agents;
        std::vector<int> collision_agents;


        // Additional private methods go here
        void invalidate_agent_cache();

        glm::dvec3 torus_adj(glm::dvec3 &pos);

        bool out_of_bounds(const glm::dvec3 &pos) const;

        void buildAgentCache();

        std::vector<Agent *> getNeighborhoodFromList(const glm::dvec3 &pos, double radius,
                                                     std::vector<int> &agent_index_search_list,
                                                     bool include_center = false);


    };
} // bhs_game
