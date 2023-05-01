//
// Created by Zartris on 4/8/23.
//

#pragma once

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include "unordered_map"
#include <random>
#include "bhs_game/agents/Agent.hpp"

namespace bhs_game {

    class BaseScheduler {
    protected:
        // Variables
        double current_time = 0.0;
        std::map<int, Agent *> agents;
        int step_count = 0;
        bool shuffled;
        std::mt19937 &random;
    public:

    public:
        BaseScheduler(bool shuffled, std::mt19937 &random) : shuffled(shuffled), random(random) {}

        ~BaseScheduler() = default;

        virtual void step(double dt);

        virtual void addAgent(Agent *agent);

        virtual void removeAgent(Agent *agent);

        int getAgentCount() const;

        [[nodiscard]] std::vector<Agent *> getAllStepAbleAgents() const;

    protected:
        static std::vector<Agent *> getMapValues(std::map<int, Agent *> m);

    private:
        [[nodiscard]] std::vector<Agent *> agentBuffer() const;

    };

    class TypeOrderedScheduler : public BaseScheduler {
    private:
        std::vector<std::string> step_order;
        std::unordered_set<std::string> all_agent_types;
        std::unordered_map<std::string, std::map<int, Agent *>> agents_by_type;
    public:
        TypeOrderedScheduler(bool shuffled, std::mt19937 &random) : BaseScheduler(shuffled, random) {};

        ~TypeOrderedScheduler() = default;

        void step(double dt) override;

        void addAgent(Agent *agent) override;

        void removeAgent(Agent *agent) override;

        void setStepOrder(std::vector<std::string> order);

    private:
        [[nodiscard]] std::vector<Agent *> agentBuffer(std::string type) const;

    };

} // bhs_game