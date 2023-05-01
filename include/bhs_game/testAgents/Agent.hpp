#pragma once


#include <utility>

#include "Eigen/Dense"
#include "bhs_game/utils/global_device.h"
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <string>

namespace bhs_game {
    class TorchAgent {
    public:
        std::string className = "Agent";
        bool hasCollision;
        bool isStatic;
    public:
        explicit TorchAgent(int unique_id, std::string className, bool hasCollision, bool isStatic) :
                uniqueId(unique_id), className(std::move(className)), hasCollision(hasCollision), isStatic(isStatic) {};

        virtual ~TorchAgent() = default;

        // The tick function should be implemented by all derived classes
        virtual void step(double dt) = 0;

        [[nodiscard]] int getUniqueId() const {
            return uniqueId;
        }

        [[nodiscard]] torch::Tensor &getState() {
            return state;
        }

        void setState(torch::Tensor state) {
            if (this->state.sizes().size() != state.sizes().size()) {
                throw std::runtime_error("State size mismatch");
            }
            if (this->state[0].sizes() != state[0].sizes()) {
                throw std::runtime_error("State size inner mismatch");
            }
            // Make sure not to overwrite state as we want the same pointers but different values.
            this->state.copy_(state);
        }

        [[maybe_unused]] torch::Tensor getOrientation() {
            return state.index({2});
        }

        [[maybe_unused]] torch::Tensor getX() {
            return state.index({0});
        }

        [[maybe_unused]] torch::Tensor getY() {
            return state.index({1}); // Note to me .item<double>(); gets the value, but is moving from gpu to cpu
        }

        torch::Tensor getPos() {
            return state.slice(0, 0, 2);
        }

        void setPos(torch::Tensor pos) {
            state.index_put_({torch::indexing::Slice(0, 2)}, pos);
        }

        void setOrientation(double orientation) {
            state.index_put_({2},
                             torch::tensor(orientation, torch::TensorOptions().dtype(torch::kDouble).device(device)));
        }

        void setX(double x) {
            state.index_put_({0}, torch::tensor(x, torch::TensorOptions().dtype(torch::kDouble).device(device)));
        }

        void setY(double y) {
            state.index_put_({1}, torch::tensor(y, torch::TensorOptions().dtype(torch::kDouble).device(device)));
        }

    protected:
        int uniqueId;
        torch::Tensor state = torch::zeros({3, 1}, torch::TensorOptions().dtype(torch::kDouble).device(device));
    };

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

    class GLMAgent {
    public:
        std::string className = "GLMAgent";
        bool hasCollision;
        bool isStatic;
    public:
        explicit GLMAgent(int unique_id, std::string className, bool hasCollision, bool isStatic) :
                uniqueId(unique_id), className(std::move(className)), hasCollision(hasCollision), isStatic(isStatic) {};

        virtual ~GLMAgent() = default;

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

        glm::dvec2 getPos() const {
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

