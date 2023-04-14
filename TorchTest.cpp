#include "torch/torch.h"
#include "bhs_game/models/AirportLogistic.hpp"
#include <chrono>
#include "bhs_game/robotics/PIDController.h"
#include <boost/stacktrace.hpp>
#include <thread>

void testSharedMemoryMatrix() {
    torch::Tensor a = torch::tensor({{1, 2, 3}});
    torch::Tensor b = torch::tensor({{4, 5, 6}});

    // Create a tensor c by stacking a and b
    torch::Tensor c = torch::stack({a, b});

    // Create views of c that share data with a and b
    torch::Tensor a_view = c.as_strided(a.sizes(), {1, 1});
    torch::Tensor b_view = c.as_strided(b.sizes(), {1, 1}, {3});

    std::cout << "c: " << c << std::endl;
    c.mul_(2);
    std::cout << "c: " << c << std::endl;
    std::cout << "a: " << a << std::endl;
    std::cout << "b: " << b << std::endl;
    std::cout << "a_view: " << a_view << std::endl;
    std::cout << "b_view: " << b_view << std::endl;

    a_view.mul_(3);
    std::cout << "c: " << c << std::endl;
    auto slice = torch::indexing::Slice(0, 2); // form 0 to 2 (not included)
    b_view.index_put_({0, torch::indexing::Slice(0, 2)}, // [0, 0:2]
                      torch::tensor({{1, 2}}));
    std::cout << "c: " << c << std::endl;
}

void pid_test_failed() {
    int N = 1;
    torch::Tensor PositionMatrix = torch::rand({N, 3, 1}, TOptions(torch::kDouble, device));
    std::cout << "PositionMatrix: " << PositionMatrix << std::endl;
    std::vector<bhs_game::AAGV *> A;
    A.reserve(N);
    int offset = 0;
    for (int i = 0; i < N; i++) {
        torch::Tensor robot_pos = PositionMatrix.as_strided({3, 1}, {1, 1}, {offset});
        offset += 3;
        std::cout << "robot_pos: " << robot_pos << std::endl;
        auto a1 = new bhs_game::AAGV(0, {100, 100},
                                     bhs_game::MotorConfig(1, 1, 1, 0.1),
                                     0.1, 0.5);
        A.push_back(a1);
        a1->overwriteState(robot_pos);
        std::cout << "a1: " << a1->getState() << std::endl;
    }

    PositionMatrix.mul_(10);
    std::cout << "PositionMatrix: " << PositionMatrix << std::endl;
    for (auto a: A) {
        std::cout << "a1: " << a->getState() << std::endl;
    }

    for (auto a: A) {
        a->setState(torch::tensor({{1},
                                   {1},
                                   {0}}, TOptions(torch::kDouble, device)));
        std::cout << "a1: " << a->getState() << std::endl;
    }
    std::cout << "================================================================================" << std::endl;
    std::cout << "========================       VECTOR PID      =================================" << std::endl;
    std::cout << "================================================================================" << std::endl;

    torch::Tensor goals = torch::full({N, 2}, {0}, TOptions(torch::kDouble, device));
    std::cout << "goals: \n" << goals << std::endl;
    std::cout << "PositionMatrix: \n" << PositionMatrix << std::endl;
    double scale = 2.;
    bhs_game::PIDController controller = bhs_game::PIDController(0.4 * scale, 0. * scale, 0.01 * scale,
                                                                 2. * scale, 0.01 * scale, 0. * scale, N);
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; i++) {
        torch::Tensor update = controller.update(PositionMatrix, goals);
        std::cout << "update: " << update << std::endl;
        for (int j = 0; j < N; j++) {
            auto a = A[j];
            a->setLinearVelocity(update.index({j, 0}));
            a->setAngularVelocity(update.index({j, 1}));
            a->inverse_kinematics();
            a->step(0.01);
            std::cout << "wheel_speed: \n" << a->getWheelSpeed() << std::endl;
        }
        std::cout << i << ": PositionMatrix: \n" << PositionMatrix << std::endl;
        std::cout << "================================================================================"
                  << std::endl;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: "
              << duration.count() << " microseconds" << std::endl;

}

// create main function
int main() {
    try {
        printf("The main function is running \n");
//        device = torch::kCUDA;
//    auto a1 = bhs_game::AAGV(0, {100, 100},
//                             bhs_game::MotorConfig(1, 1, 1, 1),
//                             1, 1);
//
//    auto a2 = bhs_game::AAGV(1, {100, 100},
//                             bhs_game::MotorConfig(1, 1, 1, 1),
//                             1, 1);

        int N = 1000;
        int testRounds = 100;

//    testSharedMemoryMatrix();

        // Option one, when initializing the model we have to make a matrix C with all the pose and then slide it after to give to the robots:
        torch::Tensor PositionMatrix = torch::ones({N, 3, 1}, TOptions(torch::kDouble, device));
//        std::cout << "PositionMatrix: " << PositionMatrix << std::endl;
        std::vector<bhs_game::AAGV *> A;
        A.reserve(N);
        int offset = 0;
        for (int i = 0; i < N; i++) {
            torch::Tensor robot_pos = PositionMatrix.as_strided({3, 1}, {1, 1}, {offset});
            offset += 3;
//            std::cout << "robot_pos: " << robot_pos << std::endl;
            auto a1 = new bhs_game::AAGV(0, {100, 100},
                                         bhs_game::MotorConfig(1, 1, 1, 0.1),
                                         0.1, 0.5);
            A.push_back(a1);
            a1->overwriteState(robot_pos);
//            std::cout << "a1: " << a1->getState() << std::endl;
        }
//
//        PositionMatrix.mul_(10);
//        std::cout << "PositionMatrix: " << PositionMatrix << std::endl;
//        for (auto a: A) {
//            std::cout << "a1: " << a->getState() << std::endl;
//        }

        for (auto a: A) {
            a->setState(torch::tensor({{1},
                                       {1},
                                       {0}}, TOptions(torch::kDouble, device)));
//            std::cout << "a1: " << a->getState() << std::endl;
        }
        std::cout << "================================================================================" << std::endl;
        std::cout << "========================       VECTOR PID      =================================" << std::endl;
        std::cout << "================================================================================" << std::endl;

        torch::Tensor goals = torch::full({N, 2}, {0}, TOptions(torch::kDouble, device));
        std::cout << "goals: \n" << goals << std::endl;
        std::cout << "PositionMatrix: \n" << PositionMatrix << std::endl;
        double scale = 2.;
        const int num_threads = 8; // number of threads to use
        const int chunk_size = N / num_threads; // number of elements to process per thread

        std::vector<std::thread> threads; // vector to hold threads
        bhs_game::PIDController controller = bhs_game::PIDController(0.4 * scale, 0. * scale, 0.01 * scale,
                                                                     2. * scale, 0.01 * scale, 0. * scale, N);
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < testRounds; i++) {
            torch::Tensor update = controller.update(PositionMatrix, goals);
//            std::cout << "update: " << update << std::endl;
//            for (int j = 0; j < N; j++) {
//                auto a = A[j];
//                a->setLinearVelocity(update.index({j, 0}));
//                a->setAngularVelocity(update.index({j, 1}));
//                a->inverse_kinematics();
//                a->step(0.01);
////                std::cout << "wheel_speed: \n" << a->getWheelSpeed() << std::endl;
//            }
            // spawn threads
            for (int t = 0; t < num_threads; t++) {
                threads.emplace_back([=] {
                    int start = t * chunk_size;
                    int end = (t == num_threads - 1) ? N : (t + 1) * chunk_size;
                    for (int j = start; j < end; j++) {
                        auto a = A[j];
                        a->setLinearVelocity(update.index({j, 0}));
                        a->setAngularVelocity(update.index({j, 1}));
                        a->inverse_kinematics();
                        a->step(0.01);
                    }
                });
            }

            // join threads
            for (auto& thread : threads) {
                thread.join();
            }
            threads.clear(); // clear thread vector for next iteration
//            std::cout << i << ": PositionMatrix: \n" << PositionMatrix << std::endl;
//            std::cout << "================================================================================" << std::endl;
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "Time taken by function: "
                  << duration.count() / testRounds << " milliseconds" << std::endl;



        // Option two, loop over all positions and make the matrix everytime:



        // Option three, give up on matrix and iterate over all the robots and update their position one by one.


        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        std::cerr << boost::stacktrace::stacktrace() << std::endl;
        return 1;
    }
}