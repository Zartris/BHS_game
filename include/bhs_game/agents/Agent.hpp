//
// Created by zartris on 4/8/23.
//

#ifndef BHS_VIS_AGENT_H
#define BHS_VIS_AGENT_H

#include "Eigen/Core"

namespace bhs_game {
    class Agent {
    public:
        explicit Agent(int unique_id);

        virtual ~Agent() = default;

        // The tick function should be implemented by all derived classes
        virtual void step(double dt) = 0;

        [[nodiscard]] int get_unique_id() const {
            return unique_id;
        }

        [[nodiscard]] Eigen::Matrix<double, 3, 1> getState() const {
            return state;
        }

        void setState(Eigen::Matrix<double, 3, 1> state) {
            this->state = state;
        }

    protected:
        int unique_id;
        Eigen::Matrix<double, 3, 1> state = Eigen::Vector3d::Zero();

    };
} // bhs_game
#endif //BHS_VIS_AGENT_H
