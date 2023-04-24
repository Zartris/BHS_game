#ifndef BHS_GAME_EAGENT_H
#define BHS_GAME_EAGENT_H

#include <utility>

#include "Eigen/Dense"
#include "bhs_game/utils/global_device.h"

namespace bhs_game {
    class EigenAgent {
    public:
        std::string className = "EigenAgent";
        bool hasCollision;
        bool isStatic;
    public:
        explicit EigenAgent(int unique_id, std::string className, bool hasCollision, bool isStatic) :
                uniqueId(unique_id), className(std::move(className)), hasCollision(hasCollision), isStatic(isStatic) {};

        virtual ~EigenAgent() = default;

        // The tick function should be implemented by all derived classes
        virtual void step(double dt) = 0;

        [[nodiscard]] int getUniqueId() const {
            return uniqueId;
        }

        [[nodiscard]] Eigen::Vector3d &getState() {
            return state; // Since state is a map, it's already an Eigen::Vector3d.
        }

        void setState(const Eigen::Vector3d &new_state) {
            state = new_state; // Assigning a new state to the map.
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

        Eigen::Vector2d getPos() const {
            return state.head<2>();
        }

        void setPos(const Eigen::Vector2d &pos) {
            state.head<2>() = pos;
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
        Eigen::Vector3d state = Eigen::Vector3d::Zero(); // Optimization: Replaced torch::Tensor with Eigen::Vector3d
    };
} // bhs_game
#endif //BHS_GAME_AGENT_H
