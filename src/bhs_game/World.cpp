//
// Created by zartris on 4/8/23.
//

#include "bhs_game/World.hpp"

namespace bhs_game {
    ContinuousSpace::ContinuousSpace(double x_max, double y_max, bool torus, double x_min, double y_min) :
            x_min(x_min), x_max(x_max), width(x_max - x_min), y_min(y_min), y_max(y_max), height(y_max - y_min),
            torus(torus) {
        center << (x_max + x_min) / 2, (y_max + y_min) / 2;
        size << width, height;
    }

    void ContinuousSpace::place_agent(Agent *agent, const Eigen::Vector2d &pos) {
        invalidate_agent_cache();
        agent_to_index[agent] = NULL; // Might set it to 0, which is not what we want.
        Eigen::Vector2d adjusted_pos = torus_adj(pos);
        agent->pos = adjusted_pos;
    }

    void ContinuousSpace::move_agent(Agent *agent, const Eigen::Vector2d &pos) {
        Eigen::Vector2d adjusted_pos = torus_adj(pos);
        agent->pos = adjusted_pos;

        if (!agent_points.empty()) {
            size_t idx = agent_to_index[agent];
            agent_points[idx] = adjusted_pos;
        }
    }

    void ContinuousSpace::remove_agent(Agent *agent) {
        if (agent_to_index.find(agent) == agent_to_index.end()) {
            throw std::runtime_error("Agent does not exist in the space");
        }
        agent_to_index.erase(agent);
        invalidate_agent_cache();
        agent->pos = Eigen::Vector2d::Zero();
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
    Eigen::Vector2d ContinuousSpace::torus_adj(const Eigen::Vector2d &pos) {

        if (!out_of_bounds(pos))
            return pos;
        else if (!torus) {
            throw std::runtime_error("Point out of bounds, and space non-toroidal.");
        } else {
            double x = x_min + fmod(pos[0] - x_min, width);
            double y = y_min + fmod(pos[1] - y_min, height);
            return {x, y};
        }
    }

    bool ContinuousSpace::out_of_bounds(const Eigen::Vector2d &pos) const {
        if (pos[0] < x_min || pos[0] > x_max || pos[1] < y_min || pos[1] > y_max) {
            return true;
        }
        return false;
    }

    void ContinuousSpace::build_agent_cache() {
        index_to_agent.clear();
        for (int i = 0; i < agent_to_index.size(); i++) {
            auto agent = index_to_agent[i];
            index_to_agent[i] = agent;
            agent_to_index[agent] = i;

        }
//        idx, agent in enumerate(self._agent_to_index):
//        self._agent_to_index[agent] = idx
//        self._index_to_agent[idx] = agent
//# Since dicts are ordered by insertion, we can iterate through agents keys
//        self._agent_points = np.array([agent.pos for agent in self._agent_to_index])
    }


} // bhs_game