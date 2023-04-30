//
// Created by zartris on 4/8/23.
//

#include "bhs_game/World.hpp"

namespace bhs_game {
    ContinuousSpace::ContinuousSpace(TScalarDouble x_max, TScalarDouble y_max, bool torus, TScalarDouble x_min,
                                     TScalarDouble y_min) :
            x_min(x_min), x_max(x_max), width(x_max - x_min), y_min(y_min), y_max(y_max), height(y_max - y_min),
            torus(torus) {
        center = torch::stack({(x_max + x_min) / 2, (y_max + y_min) / 2});
        size = torch::stack({width, height});
    }

    void ContinuousSpace::placeAgent(Agent *agent, Tensor3Double pos) {
        if (pos.sizes().size() == 1) {
            // We need it to be {3,1}
            pos = pos.unsqueeze(1);
        }
        invalidate_agent_cache();
        agent_to_index[agent] = -1; // Might set it to 0, which is not what we want.
        Tensor3Double adjusted_pos = torus_adj(pos);
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

    void ContinuousSpace::moveAgent(Agent *agent, Tensor3Double &pos) {
        Tensor3Double adjusted_pos = torus_adj(pos);
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
    Tensor3Double ContinuousSpace::torus_adj(TensorXDouble &pos) {
        if (!out_of_bounds(pos))
            return pos;
        else if (!torus) {
            throw std::runtime_error("Point out of bounds, and space non-toroidal.");
        }

        pos.index_put_({0}, x_min + torch::fmod(pos[0] - x_min, width));
        pos.index_put_({1}, y_min + torch::fmod(pos[1] - y_min, height));

        return pos;
    }

    bool ContinuousSpace::out_of_bounds(const TensorXDouble &pos) const {

        auto condition = pos.index({0}).lt(x_min).logical_or(pos.index({0}).gt(x_max)).logical_or(
                pos.index({1}).lt(y_min).logical_or(pos.index({1}).gt(y_max)));
        return condition.any().item<bool>();
    }

    void ContinuousSpace::buildAgentCache() {
        index_to_agent.clear();
        for (int i = 0; i < agent_to_index.size(); i++) {
            auto agent = index_to_agent[i];
            index_to_agent[i] = agent;
            agent_to_index[agent] = i;
        }
    }

    std::vector<Agent *> ContinuousSpace::getNeighborhoodFromList(const TensorXDouble &pos, double radius,
                                                                  std::vector<int> &agent_index_search_list,
                                                                  bool include_center) {

        if (agent_points.empty()) {
            buildAgentCache();
        }

        TensorXDouble agentsPos = torch::empty({static_cast<int64_t>(agent_index_search_list.size()), 2});

        for (size_t i = 0; i < agent_index_search_list.size(); ++i) {
            // Learning notes to myself:
            // my_target.select(0, 1).copy_(my_source.slice(1, 0, 10) would be the equivalent of
            // my_target[1] = my_source[:, :10] in Python (works if the dimensions these sub-tensors are compatible).
            agentsPos.select(0, i).copy_(index_to_agent[agent_index_search_list[i]]->getPos());
        }

        TensorXDouble deltasX = (agentsPos.index({"...", 0}) - pos.index({0})).abs();
        TensorXDouble deltasY = (agentsPos.index({"...", 1}) - pos.index({1})).abs();

        if (torus) {
            deltasX = torch::min(deltasX, size.index({0}) - deltasX);
            deltasY = torch::min(deltasY, size.index({1}) - deltasY);
        }

        TensorXDouble dists = deltasX.square() + deltasY.square();
        dists.index({0}).le(radius * radius);

        auto indx_close = std::vector<torch::Tensor>();
        // This includes one self
        if (include_center) {
            indx_close = torch::where(dists.le(radius * radius));
        } else {
            // This excludes self
            indx_close = torch::where(dists.gt(0).logical_and(dists.le(radius * radius)));
        }

        // TODO:: test if it is faster to use a vector of tensors, or a single tensor
        // Stack the vector of tensors along a new dimension to create a single tensor
        torch::Tensor indx_close_tensor = torch::stack(indx_close, 0);

        // Move the entire Tensor to CPU
        indx_close_tensor = indx_close_tensor.to(torch::kCPU);
        std::vector < Agent * > neighbors;
//        for (auto indx: indx_close) {
//            // TODO:: Be more performant, here we are having a vector with gpu tensor, and then we are converting to cpu one by one
//            int i = indx.item<int>();
//            std::cout << "Agent " << i << " is close" << std::endl;
//            neighbors.push_back(index_to_agent[agent_index_search_list[i]]);
//        }
        for (int64_t i = 0; i < indx_close_tensor.size(0); ++i) {
            int index = indx_close_tensor[i].item<int>();
            auto agent = index_to_agent[agent_index_search_list[index]];
            std::cout << "Agent " << agent->getUniqueId() << " is close" << std::endl;
            neighbors.push_back(agent);
        }

        return neighbors;
    }


} // bhs_game