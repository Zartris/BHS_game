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

    void ContinuousSpace::placeAgent(Agent *agent, const Eigen::Vector3d &pos) {
        invalidate_agent_cache();
        agent_to_index[agent] = NULL; // Might set it to 0, which is not what we want.
        Eigen::Vector3d adjusted_pos = torus_adj(pos);
        agent->setState(adjusted_pos);

        if (agent->isStatic) {
            static_agents.push_back(agent->getUniqueId());
        } else {
            dynamic_agents.push_back(agent->getUniqueId());
        }
        if (agent->hasCollision) {
            collision_agents.push_back(agent->getUniqueId());
        }
    }

    void ContinuousSpace::moveAgent(Agent *agent, const Eigen::Vector3d &pos) {
        Eigen::Vector3d adjusted_pos = torus_adj(pos);
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
        agent->setPos(Eigen::Vector2d::Zero());

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
    Eigen::Vector3d ContinuousSpace::torus_adj(const Eigen::Vector3d &pos) {
        if (!out_of_bounds(pos))
            return pos;
        else if (!torus) {
            throw std::runtime_error("Point out of bounds, and space non-toroidal.");
        }
        double x = x_min + fmod(pos[0] - x_min, width);
        double y = y_min + fmod(pos[1] - y_min, height);
        return {x, y, pos[2]};
    }

    bool ContinuousSpace::out_of_bounds(const Eigen::Vector3d &pos) const {
        if (pos[0] < x_min || pos[0] > x_max || pos[1] < y_min || pos[1] > y_max) {
            return true;
        }
        return false;
    }

    void ContinuousSpace::buildAgentCache() {
        index_to_agent.clear();
        for (int i = 0; i < agent_to_index.size(); i++) {
            auto agent = index_to_agent[i];
            index_to_agent[i] = agent;
            agent_to_index[agent] = i;
        }
    }

    std::vector<Agent *> ContinuousSpace::getNeighborhoodFromList(const Eigen::Vector3d &pos, double radius,
                                                                  std::vector<int> &agent_index_search_list,
                                                                  bool include_center) {

        if (agent_points.empty()) {
            buildAgentCache();
        }

        Eigen::MatrixXd agentsPos(agent_index_search_list.size(), 2);
        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            agentsPos.row(i) = index_to_agent[agent_index_search_list[i]]->getPos();
        }

        Eigen::ArrayXd deltasX = (agentsPos.col(0).array() - pos[0]).abs();
        Eigen::ArrayXd deltasY = (agentsPos.col(1).array() - pos[1]).abs();

        if (torus) {
            deltasX = deltasX.min(size - deltasX);
            deltasY = deltasY.min(size - deltasY);
        }

        Eigen::ArrayXd dists = deltasX.square() + deltasY.square();

        std::vector<Agent *> neighbors;
        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            if (dists[i] <= radius * radius) {
                if (include_center || dists[i] > 0) {
                    neighbors.push_back(index_to_agent[agent_index_search_list[i]]);
                }
            }
        }

        return neighbors;
    }


} // bhs_game