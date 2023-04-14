//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_AGENT_H
#define BHS_GAME_AGENT_H

#include <utility>

#include "torch/torch.h"
#include "bhs_game/utils/global_device.h"

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
} // bhs_game
#endif //BHS_GAME_AGENT_H
