//
// Created by ohmy on 2022-03-14.
//

//
// Created by ohmy on 2022-02-22.
//
#include <mropt/mropt.h>
#include "mropt/Robot/BuilderDistributedRobot_Dubins.h"
#include "mropt/Robot/BuilderDistributedRobot_Unicycle.h"

int main(int argc, char **argv) {

    // Initialisation OPENMPI
    MPI_Init(&argc, &argv);
    int r, size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    MPI_Comm_rank(MPI_COMM_WORLD, &r);

    // 0 - Maps
    std::string maps_path = "../maps/";
    std::string map_file = "0.5pol.png";

    // 1 - Mission
    int N = 50;
    double T = 10;

/*
    // 1.1) Starting Configuration (x, y, theta) for 3 robots
    std::vector<std::vector<double>> start =
            {
                    {1, 1, -0.5 * M_PI},
                    {3, 4, 0.5 * M_PI},
                    {1, 4, 0}
            };
    // 1.2) Goal Configuration (x,y, theta) for 3 robot
    std::vector<std::vector<double>> goal =
            {
                    {2, 2.5, 0},
                    {3, 2,   0},
                    {4, 1,   0}
            };
    // 1.3) Length and Width
    std::vector<double> L = {1,
                             0.6,
                             1.3};
    std::vector<double> W = {1, 0.6, 1.3};
*/

    // 1.1) Starting Configuration (x, y, theta) for 3 robots
    std::vector<std::vector<double>> start =
            {
                    {1, 1, 0.5 * M_PI},
                    {1, 4, 0}
            };
    // 1.2) Goal Configuration (x,y, theta) for 3 robot
    std::vector<std::vector<double>> goal =
            {
                    {2, 2.5, 0},
                    {4, 1,   0}
            };
    // 1.3) Length and Width
    std::vector<double> L = {1,0.5};
    std::vector<double> W = {1, 0.5};



    // Test MPI treads is equal to number of robots
    int R = start.size();
    if (size != R) {
        std::cerr << "Decoupled case must have R mpi thread!" << std::endl;
    }

    // 2 - Build a Robot
    mropt::Problem::BuilderDistributedRobot_Unicycle builder_unicycle_car(L[r]);
    auto params = mropt::Problem::Robot::Params{0.0, T, N};
    builder_unicycle_car.make_robot(r, params);
    auto robot_d = builder_unicycle_car.getDistributedRobot();
/*    mropt::Problem::BuilderDistributedRobot_Dubins builder_dubins_car(L[r]);
    auto params = mropt::Problem::Robot::Params{0.0, T, N};
    builder_dubins_car.make_robot(r, params);
    auto robot_d = builder_dubins_car.getDistributedRobot();*/
    // 3.2 - Assign missions to the robots
    robot_d->addMission(start[r], goal[r], {});

    // 4 -Solve
    // 4.1 - Setup Decoupled Plotter and Solver
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
    //try {
        mrprob_d.solve();
        mrprob_d.plot_trajectories(std::vector<std::shared_ptr<mropt::Dynamics::ode>>(R, robot_d->get_ode()));
        MPI_Barrier(MPI_COMM_WORLD);
/*
    } catch (...) {
        std::exit(1); // terminate with exit code 1 = fail
    }
*/

}