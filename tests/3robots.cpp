//
// Created by ohmy on 2022-02-22.
//
#include <mropt/mropt.h>

int main(int argc, char** argv) {

// Initialisation OPENMPI
    MPI_Init(&argc, &argv);
    int r, size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    MPI_Comm_rank(MPI_COMM_WORLD, &r);

// 0 - Maps
    std::string maps_path = "../maps/";
    std::string map_file = "0.5pol.png";

// 1 - Mission
    int N = 30;
    double T = 6;
    std::vector<std::vector<double>> start =
            {
                { 1, 1, -0.5*M_PI},
                { 3 , 4, 0.5*M_PI},
                { 1, 4, 0}
            };
    std::vector<std::vector<double>> goal =
            {
                    { 2, 2.5, 0},
                    { 3 , 2, 0},
                    { 4, 1, 0}
            };
    std::vector<double>  L = {1, 0.6, 1.3};
    std::vector<double>  W = {1, 0.6, 1.3};


// Decoupled case threads test
    int R = start.size();
    if (size != R) {
        std::cerr << "Decoupled case must have R mpi thread!" << std::endl;
    }

// 1 - Initialize robots
//State Space
    auto se2 = std::make_shared<mropt::StateSpace::SE2>();
//Control Space
    auto vw = std::make_shared<mropt::ControlSpace::VW>();
// Dynamic Model
    auto model = std::make_shared<mropt::Dynamics::CarLike::DiffDrive>(vw, se2, L[r]);
//Robot Shape
    auto shape = std::make_shared<mropt::RobotShape::CircleRobot>(L[r] / 2);
// Model Approx
    auto fot = std::make_shared<mropt::Dynamics::Approx::FirstOrderTaylor>(model);
// Transcription Approx
    auto ms_approx = std::make_shared<mropt::Dynamics::MultipleShootingApprox>(fot);
// Cost
    auto cost = std::make_shared<mropt::cost::Cost>(*vw, *se2);
// Free Space
    auto cfree = std::make_shared<mropt::freespace::FreeSpace>(*se2);
// Collisions Approximation - Coupled
    auto collision_type = mropt::collisions::L2NormCollision();
// Collisions Augmented Lagrangian - ADMM
    auto dis_col = std::make_shared<mropt::collisions::FirstOrderTaylorDistributedCollisions>(collision_type);
// Robot
    auto params = mropt::Problem::Robot::Params{0.0, T, N};
    auto robot_d = std::make_shared<mropt::Problem::DistributedRobot>(r, params, vw, se2, cost, cfree, ms_approx, shape,
                                                                      dis_col);


// 3.2 - Assign missions to the robots
    robot_d->addMission(start[r], goal[r], {});

// 4 -Solve
// 4.1 - Setup Decoupled Solver
    auto plotter = std::make_shared<mropt::util::Opencv_plotter>(R); // dummy
    plotter->setFootprint({L}, {W});
    plotter->addPathToScenarios(maps_path);
    plotter->inputScenario(map_file);

    mropt::Problem::DecoupledProblem mrprob_d{r};
    mrprob_d.setParams(R, N);
    mrprob_d.addRobot(robot_d);
    mrprob_d.set_plotter(plotter);
    mrprob_d.debug_mode().allow_plotting();
// 4.2 - Solve
    try {
        mrprob_d.solve();
        mrprob_d.plot_trajectories(std::vector<std::shared_ptr<mropt::Dynamics::ode>>(R, robot_d->get_ode()));
        MPI_Barrier(MPI_COMM_WORLD);
    } catch (...) {
        std::exit(1); // terminate with exit code 1 = fail
    }

}