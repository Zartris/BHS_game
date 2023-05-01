//
// Created by zartris on 4/8/23.
//

#pragma once

#include "glm/glm.hpp"
#include "glm/gtx/norm.hpp"
#include "string"

namespace bhs_game {
    class Agent {
    public:
        std::string className = "Agent";
        bool hasCollision;
        bool isStatic;
    public:
        explicit Agent(int unique_id, std::string className, bool hasCollision, bool isStatic) :
                uniqueId(unique_id), className(std::move(className)), hasCollision(hasCollision), isStatic(isStatic) {};

        virtual ~Agent() = default;

        // The tick function should be implemented by all derived classes
        virtual void step(double dt) = 0;

        [[nodiscard]] int getUniqueId() const {
            return uniqueId;
        }

        [[nodiscard]] glm::dvec3 &getState() {
            return state;
        }

        void setState(const glm::dvec3 &new_state) {
            state = new_state;
        }

        [[maybe_unused]] double &getOrientation() {
            return state[2];
        }

        [[maybe_unused]] double &getX() {
            return state[0];
        }

        [[maybe_unused]] double &getY() {
            return state[1];
        }

        [[nodiscard]] glm::dvec2 getPos() const {
            return glm::dvec2(state.x, state.y);
        }

        void setPos(const glm::dvec2 &pos) {
            state.x = pos.x;
            state.y = pos.y;
        }

        void setOrientation(double orientation) {
            state[2] = orientation;
        }

        void setX(double x) {
            state[0] = x;
        }

        void setY(double y) {
            state[1] = y;
        }

    protected:
        int uniqueId;
        glm::dvec3 state = glm::dvec3(0.0); // Optimization: Replaced Eigen::Vector3d with glm::dvec3
    };
} // bhs_game
