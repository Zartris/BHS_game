//
// Created by zartris on 4/8/23.
//

#include "bhs_game/World.hpp"

namespace bhs_game {
    ContinuousSpace::ContinuousSpace(double x_max, double y_max, bool torus, double x_min,
                                     double y_min) :
            x_min(x_min), x_max(x_max), width(x_max - x_min), y_min(y_min), y_max(y_max), height(y_max - y_min),
            torus(torus), center(glm::dvec2((x_max + x_min) / 2, (y_max + y_min) / 2)),
            size(glm::dvec2(width, height)) {
    }

    void ContinuousSpace::placeAgent(Agent *agent, glm::dvec3 pos) {
        invalidate_agent_cache();
        agent_to_index[agent] = -1; // Might set it to 0, which is not what we want.
        glm::dvec3 adjusted_pos = torus_adj(pos);
        agent->setState(adjusted_pos);
        // TODO:: Maybe make an agent pos matrix for vectorized operations
        if (agent->isStatic) {
            static_agents.push_back(agent->getUniqueId());
        } else {
            dynamic_agents.push_back(agent->getUniqueId());
        }
        if (agent->hasCollision) {
            collision_agents.push_back(agent->getUniqueId());
        }
    }

    void ContinuousSpace::moveAgent(Agent *agent, glm::dvec3 &pos) {
        glm::dvec3 adjusted_pos = torus_adj(pos);
        agent->setState(adjusted_pos);

        if (!agent_points.empty()) {
            size_t idx = agent_to_index[agent];
            agent_points[idx] = adjusted_pos;
        }
    }

    void ContinuousSpace::removeAgent(Agent *agent) {
        if (agent_to_index.find(agent) == agent_to_index.end()) {
            throw std::runtime_error("Agent does not exist in the space");
        }
        agent_to_index.erase(agent);
        invalidate_agent_cache();

        // remove from static/dynamic/collision lists
        if (agent->isStatic) {
            auto indx = std::find(static_agents.begin(), static_agents.end(), agent->getUniqueId());
            if (indx != static_agents.end()) {
                static_agents.erase(indx);
            }
        } else {
            auto indx = std::find(dynamic_agents.begin(), dynamic_agents.end(), agent->getUniqueId());
            if (indx != dynamic_agents.end()) {
                dynamic_agents.erase(indx);
            }
        }
        if (agent->hasCollision) {
            auto indx = std::find(collision_agents.begin(), collision_agents.end(), agent->getUniqueId());
            if (indx != collision_agents.end()) {
                collision_agents.erase(indx);
            }
        }
    }

    void ContinuousSpace::invalidate_agent_cache() {
        agent_points.clear();
        index_to_agent.clear();
    }

    /**
     * Adjusts a position to be within the bounds of the space, if toroidal.
     * @param pos
     * @return The adjusted position.
     */
    glm::dvec3 ContinuousSpace::torus_adj(glm::dvec3 &pos) {
        if (!out_of_bounds(pos))
            return pos;
        else if (!torus) {
            throw std::runtime_error("Point out of bounds, and space non-toroidal.");
        }
        pos[0] = x_min + std::fmod(pos[0] - x_min, width);
        pos[1] = y_min + std::fmod(pos[1] - y_min, height);
        return pos;
    }

    bool ContinuousSpace::out_of_bounds(const glm::dvec3 &pos) const {
        return pos[0] < x_min || pos[0] > x_max || pos[1] < y_min || pos[1] > y_max;
    }

    void ContinuousSpace::buildAgentCache() {
        index_to_agent.clear();
        for (int i = 0; i < agent_to_index.size(); i++) {
            auto agent = index_to_agent[i];
            index_to_agent[i] = agent;
            agent_to_index[agent] = i;
        }
    }

    std::vector<Agent *> ContinuousSpace::getNeighborhoodFromList(const glm::dvec3 &pos, double radius,
                                                                  std::vector<int> &agent_index_search_list,
                                                                  bool include_center) {
        if (agent_points.empty()) {
            buildAgentCache();
        }

        std::vector<glm::dvec2> agentsPos(agent_index_search_list.size());

        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            agentsPos[i] = index_to_agent[agent_index_search_list[i]]->getPos();
        }

        std::vector<glm::dvec2> deltas(agent_index_search_list.size());
        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            deltas[i] = agentsPos[i] - pos.xy();
            if (torus) {
                deltas[i].x = std::min(deltas[i].x, size.x - deltas[i].x);
                deltas[i].y = std::min(deltas[i].y, size.y - deltas[i].y);
            }
        }

        std::vector<double> dists(agent_index_search_list.size());
        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            dists[i] = glm::length2(deltas[i]);
        }

        std::vector<Agent *> neighbors;
        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            bool is_close = (dists[i] <= radius * radius);
            bool is_not_center = (dists[i] > 0);

            if ((include_center && is_close) || (!include_center && is_close && is_not_center)) {
                neighbors.push_back(index_to_agent[agent_index_search_list[i]]);
            }
        }

        return neighbors;
    }
} // bhs_game