//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_AGENT_H
#define BHS_GAME_AGENT_H

#include <utility>

#include "Eigen/Core"

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

        [[nodiscard]] Eigen::Matrix<double, 3, 1> getState() const {
            return state;
        }

        void setState(Eigen::Matrix<double, 3, 1> state) {
            this->state = state;
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

        Eigen::Vector2d getPos() {
            return {state[0], state[1]};
        }

        void setPos(Eigen::Vector2d pos) {
            state[0] = pos[0];
            state[1] = pos[1];
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
        Eigen::Matrix<double, 3, 1> state = Eigen::Vector3d::Zero();


    };
} // bhs_game
#endif //BHS_GAME_AGENT_H
