//
// Created by zartris on 4/8/23.
//

#include "bhs_game/Scheduler.hpp"

#include <random>

namespace bhs_game {
    void BaseScheduler::addAgent(Agent *agent) {
        int unique_id = agent->getUniqueId();
        if (agents.find(unique_id) != agents.end()) {
            throw std::runtime_error(
                    "Agent with unique id " + std::to_string(unique_id) + " already added to scheduler");
        }

        agents[unique_id] = agent;
    }

    void BaseScheduler::removeAgent(Agent *agent) {
        int unique_id = agent->getUniqueId();
        agents.erase(unique_id);
    }

    void BaseScheduler::step(double dt) {
        for (auto &agent: agentBuffer()) {
            agent->step(dt);
        }
        ++step_count;
        current_time += dt;
    }

    int BaseScheduler::getAgentCount() const {
        return static_cast<int>(agents.size());
    }

    std::vector<Agent *> BaseScheduler::getAllStepAbleAgents() const {
        return getMapValues(agents);
    }

    std::vector<Agent *> BaseScheduler::getMapValues(std::map<int, Agent *> m) {
        std::vector<Agent *> agent_vec;
        agent_vec.reserve(m.size());
        for (auto &agent_pair: m) {
            agent_vec.push_back(agent_pair.second);
        }
        return agent_vec;
    }

    std::vector<Agent *> BaseScheduler::agentBuffer() const {
        std::vector<Agent *> agent_vec = getAllStepAbleAgents();
        if (shuffled) {
            std::shuffle(agent_vec.begin(), agent_vec.end(), random);
        }
        return agent_vec;
    }


    void TypeOrderedScheduler::step(double dt) {
        if (step_order.empty()) {
            printf("Step order is empty, using all agent types\n");
            // If the step order is empty, use the types of all agents
            step_order = std::vector<std::string>(all_agent_types.begin(), all_agent_types.end());
        }
        // Stepping through the agents in the order specified by step_order
        for (auto &agent_type: step_order) {
            // Extract the agents of the current type and shuffle them if needed
            std::vector<Agent *> agent_vec = getMapValues(agents_by_type[agent_type]);
            if (shuffled) {
                std::shuffle(agent_vec.begin(), agent_vec.end(), random);
            }
            // Step through the agents
            for (auto &agent: agent_vec) {
                agent->step(dt);
            }
        }
        ++step_count;
        current_time += dt;
    }


    void TypeOrderedScheduler::addAgent(Agent *agent) {
        BaseScheduler::addAgent(agent);
        all_agent_types.insert(agent->className);
        agents_by_type[agent->className][agent->getUniqueId()] = agent;
    }

    void TypeOrderedScheduler::removeAgent(Agent *agent) {
        BaseScheduler::removeAgent(agent);
        agents_by_type[agent->className].erase(agent->getUniqueId());
        if (agents_by_type[agent->className].empty()) {
            all_agent_types.erase(agent->className);
        }
    }

    void TypeOrderedScheduler::setStepOrder(std::vector<std::string> order) {
        // Check if we have all the types in the order
        for (auto &agent_type: order) {
            if (all_agent_types.find(agent_type) == all_agent_types.end()) {
                throw std::runtime_error("Agent type " + agent_type + " not found in scheduler");
            }
        }

        step_order = order;
        // Add all missing types to the end of the step order
        for (auto &agent_type: all_agent_types) {
            if (std::find(step_order.begin(), step_order.end(), agent_type) == step_order.end()) {
                step_order.push_back(agent_type);
            }
        }
    }
} // bhs_game