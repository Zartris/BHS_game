#include "torch/torch.h"
#include "bhs_game/models/AirportLogistic.hpp"
#include <chrono>
#include "bhs_game/robotics/PIDController.h"
#include <boost/stacktrace.hpp>
#include <thread>
#include "Eigen/Dense"
#include "bhs_game/testAgents/AGV.hpp"
#include <glm/gtx/string_cast.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
//#include <tbb/tbb.h>
#include <tbb/global_control.h>
//#include <tbb/task_scheduler_init.h>


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

void et_matrix_mul_test() {
    torch::NoGradGuard no_grad;
    const int size = 1000;
    std::cout << "matrix multiplication size=" << size << std::endl;

    constexpr int N = size, M = size, K = size;

    // get a random number generator between 0 and N
    std::mt19937 gen1(0);
    std::uniform_int_distribution<> dis(0, N - 1);
    int n = dis(gen1);


    // Eigen
    auto start_eigen = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(N, M);
    Eigen::MatrixXd B = Eigen::MatrixXd::Random(M, K);
    Eigen::MatrixXd C;
    C = A * B;
    double val_eigen = C(n, n);

    auto end_eigen = std::chrono::high_resolution_clock::now();
    auto duration_eigen = std::chrono::duration_cast<std::chrono::microseconds>(end_eigen - start_eigen).count();

    // Torch
    auto start_torch = std::chrono::high_resolution_clock::now();
    torch::Tensor A_torch = torch::randn({N, M});
    torch::Tensor B_torch = torch::randn({M, K});
    torch::Tensor C_torch;
    C_torch = A_torch.matmul(B_torch);
    double val_torch = C_torch.index({n, n}).item<double>();
    auto end_torch = std::chrono::high_resolution_clock::now();
    auto duration_torch = std::chrono::duration_cast<std::chrono::microseconds>(end_torch - start_torch).count();

//    std::cout << "Eigen: " << val_eigen << std::endl;
//    std::cout << "Torch: " << val_torch.item<double>() << std::endl;

    std::cout << "Eigen: " << duration_eigen << " microseconds (" << static_cast<double>(duration_eigen) * 0.001
              << " milliseconds)" << std::endl;
    std::cout << "Torch: " << duration_torch << " microseconds (" << static_cast<double>(duration_torch) * 0.001
              << " milliseconds)\n" << std::endl;
}

void et_matrix_index_test() {
    torch::NoGradGuard no_grad;
    const int size = 1000;
    std::cout << "matrix bitwise product with constants size=" << size << std::endl;
    constexpr int N = size, M = size, K = size;


    // Eigen
    auto start_eigen = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(N, M);
    Eigen::MatrixXd B = Eigen::MatrixXd::Random(N, M);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N, M);

    C = (2 * A + 3 * B).cwiseProduct(A) / 4;
    double val_eigen = C(0, 0);
    auto end_eigen = std::chrono::high_resolution_clock::now();
    auto duration_eigen = std::chrono::duration_cast<std::chrono::microseconds>(end_eigen - start_eigen).count();

    // Torch
    auto start_torch = std::chrono::high_resolution_clock::now();
    torch::Tensor A_torch = torch::randn({N, M});
    torch::Tensor B_torch = torch::randn({N, M});
    torch::Tensor C_torch;

    C_torch = (2 * A_torch + 3 * B_torch) * A_torch / 4;
    torch::Tensor val_torch = C_torch.index({0, 0});
    auto end_torch = std::chrono::high_resolution_clock::now();
    auto duration_torch = std::chrono::duration_cast<std::chrono::microseconds>(end_torch - start_torch).count();
//    std::cout << "Eigen: " << val_eigen << std::endl;
//    std::cout << "Torch: " << val_torch.item<double>() << std::endl;

    std::cout << "Eigen: " << duration_eigen << " microseconds (" << static_cast<double>(duration_eigen) * 0.001
              << " milliseconds)" << std::endl;
    std::cout << "Torch: " << duration_torch << " microseconds (" << static_cast<double>(duration_torch) * 0.001
              << " milliseconds)\n" << std::endl;



    // get a random number generator between 0 and N
    std::mt19937 gen1(0);
    std::mt19937 gen2(0);
    std::uniform_int_distribution<> dis(0, N - 1);
    int testRuns = 10000;
    std::cout << "matrix indexing: " << testRuns << std::endl;

    // time it
    start_eigen = std::chrono::high_resolution_clock::now();
    double sum1 = 0;
    for (int i = 0; i < testRuns; ++i) {
        int n = dis(gen1);
        int m = dis(gen1);

        val_eigen = C(n, m);
        sum1 += val_eigen;
    }
    end_eigen = std::chrono::high_resolution_clock::now();
    duration_eigen = std::chrono::duration_cast<std::chrono::microseconds>(end_eigen - start_eigen).count();

    start_torch = std::chrono::high_resolution_clock::now();
    double sum2 = 0;
    for (int i = 0; i < testRuns; ++i) {
        int n = dis(gen2);
        int m = dis(gen2);

        val_torch = C_torch.index({n, m});
        sum2 += val_torch.item<double>();
    }
    end_torch = std::chrono::high_resolution_clock::now();
    duration_torch = std::chrono::duration_cast<std::chrono::microseconds>(end_torch - start_torch).count();


    start_torch = std::chrono::high_resolution_clock::now();
    torch::Tensor sum3 = torch::scalar_tensor(0);
    for (int i = 0; i < testRuns; ++i) {
        int n = dis(gen2);
        int m = dis(gen2);

        val_torch = C_torch.index({n, m});
        sum3 += val_torch;
    }
    end_torch = std::chrono::high_resolution_clock::now();
    auto duration_torch_tensor = std::chrono::duration_cast<std::chrono::microseconds>(end_torch - start_torch).count();



    // print sum
//    std::cout << "Eigen: " << sum1 << std::endl;
//    std::cout << "Torch: " << sum2 << std::endl;
//    std::cout << "Torch: " << sum3.item<double>() << std::endl;

    std::cout << "Eigen: " << duration_eigen << " microseconds (" << static_cast<double>(duration_eigen) * 0.001
              << " milliseconds)" << std::endl;
    std::cout << "Torch: " << duration_torch << " microseconds (" << static_cast<double>(duration_torch) * 0.001
              << " milliseconds)" << std::endl;
    std::cout << "Torch: " << duration_torch_tensor << " microseconds ("
              << static_cast<double>(duration_torch_tensor) * 0.001 << " milliseconds)\n" << std::endl;

}

void eigen_vs_torch() {
    std::cout << "Eigen vs Torch" << std::endl;
    et_matrix_mul_test();
    et_matrix_index_test();
}

void torch_pid_test(torch::Tensor PositionMatrix, std::vector<bhs_game::TorchAAGV *> A, int N, int testRounds) {
    std::cout << "PID Torch test" << std::endl;
    torch::Tensor goals = torch::full({N, 2}, {0}, TOptions(torch::kDouble, device));
//    std::cout << "goals: \n" << goals << std::endl;
//    std::cout << "PositionMatrix: \n" << PositionMatrix << std::endl;
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
//        for (int j = 0; j < N; j++) {
//            auto a = A[j];
//            a->setLinearVelocity(update.index({j, 0}));
//            a->setAngularVelocity(update.index({j, 1}));
//            a->inverse_kinematics();
//            a->step(0.01);
////                std::cout << "wheel_speed: \n" << a->getWheelSpeed() << std::endl;
//        }
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
        for (auto &thread: threads) {
            thread.join();
        }
        threads.clear(); // clear thread vector for next iteration
//            std::cout << i << ": PositionMatrix: \n" << PositionMatrix << std::endl;
//            std::cout << "================================================================================" << std::endl;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: "
              << duration.count() / testRounds * 0.001 << " milliseconds" << std::endl;
}

void
eigen_pid_test(std::vector<bhs_game::EigenAAGV *> A, int N, int testRounds) {
    std::cout << "Eigen PID Test" << std::endl;
    Eigen::MatrixXd goals = Eigen::MatrixXd::Zero(2, N);
    Eigen::Matrix<double, 3, Eigen::Dynamic> PositionMatrix = Eigen::MatrixXd::Ones(3, N);

    const int num_threads = 8; // number of threads to use
    const int chunk_size = N / num_threads; // number of elements to process per thread

    std::vector<std::thread> threads; // vector to hold threads

    double scale = 2.;
    bhs_game::EPIDController controller = bhs_game::EPIDController(0.4 * scale, 0. * scale, 0.01 * scale,
                                                                   4. * scale, 0.01 * scale, 0. * scale, N);
    auto start = std::chrono::high_resolution_clock::now();

    // Note: Using threads added too much overhead for this test as it double the runtime
    for (int i = 0; i < testRounds; i++) {
        Eigen::MatrixXd update = controller.update(PositionMatrix, goals);
#ifdef VERBOSE
        std::cout << "\n============================" << std::endl;
        std::cout << i << " current state:\n" << A[0]->getState() << std::endl;
        std::cout << "updates (vel, ang): \n" << update << std::endl;
#endif

        tbb::parallel_for(tbb::blocked_range<size_t>(0, N), [&](const tbb::blocked_range<size_t> &r) {
            for (size_t j = r.begin(); j != r.end(); ++j) {
                auto a = A[j];
                a->setLinearVelocity(update(j, 0));
                a->setAngularVelocity(update(j, 1));
                a->inverse_kinematics(true);
                a->step(0.01);
            }
        });

//        for (int j = 0; j < N; j++) {
//            auto a = A[j];
//            a->setLinearVelocity(update(j, 0));
//            a->setAngularVelocity(update(j, 1));
//            a->inverse_kinematics(true);
//            a->step(0.01);
//            // Update Position Matrix
////            PositionMatrix.col(j) = a->getState();
//        }
        // Note: 0.001 us did it cost to split the forloop, this one is easier to read
        for (int j = 0; j < N; j++) {
            // Update Position Matrix
            PositionMatrix.col(j) = A[j]->getState();
        }

    }
#ifdef VERBOSE
    std::cout << "final: current state:\n" << A[0]->getState() << std::endl;
#endif
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//    for (int j = 0; j < N; j++) {
//        auto a = A[j];
//        std::cout << "state: \n" << a->getState() << std::endl;
//    }
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, N - 1); // distribution in range [1, 6]
    int index = dist(rng);
    std::cout << "\nPosition matrix for agent index: " << index << "\n" << PositionMatrix.col(index) << std::endl;
    std::cout << "Time taken by function: "
              << duration.count() / testRounds * 0.001 << " milliseconds" << std::endl;
}

void
glm_pid_test(std::vector<bhs_game::GLMAAGV *> A, int N, int testRounds) {
    std::cout << "GLM PID Test" << std::endl;

    double scale = 2.;
    for (auto a: A) {
        a->controller = bhs_game::GPIDController(0.4 * scale, 0. * scale, 0.01 * scale,
                                                 4. * scale, 0.01 * scale, 0. * scale);
        a->goal = glm::dvec2(0.);
        a->setState(glm::dvec3(1, 1, 0));
    }


    auto start = std::chrono::high_resolution_clock::now();

    // Note: Using threads added too much overhead for this test as it double the runtime
    for (int i = 0; i < testRounds; i++) {
#ifdef VERBOSE
        std::cout << "\n============================" << std::endl;
        std::cout << i << " current state:\n" << A[0]->getState() << std::endl;
//        std::cout << "updates (vel, ang): \n" << update << std::endl;
#endif
        tbb::parallel_for(tbb::blocked_range<size_t>(0, N), [&](const tbb::blocked_range<size_t> &r) {
            for (size_t j = r.begin(); j != r.end(); ++j) {
                auto a = A[j];
                a->step(0.01);
            }
        });
    }
#ifdef VERBOSE
    std::cout << "final: current state:\n" << A[0]->getState() << std::endl;
#endif

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
// get random index of A
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, N - 1); // distribution in range [1, 6]
    int index = dist(rng);
    std::cout << "\nPosition vector for agent index: " << index << "\n" << glm::to_string(A[index]->getState())
              << std::endl;
    std::cout << "Time taken by function: "
              << duration.count() / testRounds * 0.001 << " milliseconds" << std::endl;
}


static void BM_TorchAdd(int num) {
    torch::NoGradGuard no_grad;
    torch::Tensor tensor = torch::rand({2, 2});
    for (int i; i < num; ++i)
        auto v = tensor + tensor;
}

// create main function
int main() {
    try {
        printf("The main function is running \n");
//        device = torch::kCUDA;
        int N = 10000;
        int testRounds = 100;
        int nthread = 8;
        tbb::global_control c(tbb::global_control::max_allowed_parallelism, nthread);

//        tbb::task_scheduler_init init(nthread);
        Eigen::setNbThreads(nthread);

        auto n = Eigen::nbThreads();
        Eigen::initParallel();
        std::cout << "Eigen threads: " << n << std::endl;
//    testSharedMemoryMatrix();


        std::cout << "================================================================================" << std::endl;
        std::cout << "========================        TORCH PID      =================================" << std::endl;
        std::cout << "================================================================================" << std::endl;
        {// Option one, when initializing the model we have to make a matrix C with all the pose and then slide it after to give to the robots:
            torch::Tensor PositionMatrix = torch::ones({N, 3, 1}, TOptions(torch::kDouble, device));
            std::vector<bhs_game::TorchAAGV *> A;
            A.reserve(N);
            int offset = 0;
            for (int i = 0; i < N; i++) {
                torch::Tensor robot_pos = PositionMatrix.as_strided({3, 1}, {1, 1}, {offset});
                offset += 3;
                auto a1 = new bhs_game::TorchAAGV(0, {100, 100},
                                                  bhs_game::TMotorConfig(1, 1, 1, 0.1),
                                                  0.1, 0.5);
                A.push_back(a1);
                a1->overwriteState(robot_pos);
            }

            for (auto a: A) {
                a->setState(torch::tensor({{1},
                                           {1},
                                           {0}}, TOptions(torch::kDouble, device)));
            }
            torch_pid_test(PositionMatrix, A, N, testRounds);
        }
        std::cout << "================================================================================" << std::endl;
        std::cout << "========================       EIGEN PID      =================================" << std::endl;
        std::cout << "================================================================================" << std::endl;
        {
            std::vector<bhs_game::EigenAAGV *> Aa;
            Aa.reserve(N);
            for (int i = 0; i < N; i++) {
//            auto robot_pos = std::make_shared<Eigen::Vector3d>(EPositionMatrix.col(i));
                auto a2 = new bhs_game::EigenAAGV(0, {100, 100},
                                                  bhs_game::EMotorConfig(1, 1, 1, 0.1),
                                                  0.1, 0.5);
                Aa.push_back(a2);
                a2->setState(Eigen::Vector3d(1, 1, 0));
            }
            eigen_pid_test(Aa, N, testRounds);
        }

        std::cout << "================================================================================" << std::endl;
        std::cout << "========================         GLM PID       =================================" << std::endl;
        std::cout << "================================================================================" << std::endl;
        {
            std::vector<bhs_game::GLMAAGV *> Ab;
            Ab.reserve(N);
            for (int i = 0; i < N; i++) {
                auto a2 = new bhs_game::GLMAAGV(0, {100, 100},
                                                bhs_game::EMotorConfig(1, 1, 1, 0.1),
                                                0.1, 0.5);
                Ab.push_back(a2);
                a2->setState(glm::dvec3(1, 1, 0));
            }
            glm_pid_test(Ab, N, testRounds);
        }

        // Option two, loop over all positions and make the matrix everytime:
//        eigen_vs_torch();


        // Option three, give up on matrix and iterate over all the robots and update their position one by one.


        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        std::cerr << boost::stacktrace::stacktrace() << std::endl;
        return 1;
    }
}