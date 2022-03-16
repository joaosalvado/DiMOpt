//
// Created by ohmy on 2022-03-14.
//

//
// Created by ohmy on 2022-02-22.
//
#include <mropt/mropt.h>
#include "mropt/Robot/BuilderDistributedRobot_Dubins.h"
#include "mropt/Robot/BuilderDistributedRobot_Unicycle.h"
#include "mropt/Robot/BuilderRobot_DiffDriveQuat.h"

int main(int argc, char **argv) {

    // Initialisation OPENMPI
    MPI_Init(&argc, &argv);
    int r, size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    MPI_Comm_rank(MPI_COMM_WORLD, &r);

    // 0 - Maps
    std::string maps_path = "../maps/";
    std::string map_file = "0.5pol.png";
    // 1 polygon box
    std::vector<std::vector<std::vector<double>>> polygons
            =
            {{
                     {
                             -1.0,
                             -0.0,
                             -0.04
                     },
                     {
                             -0.0,
                             1.0,
                             4.97
                     },
                     {
                             1.0,
                             0.0,
                             5.0
                     },
                     {
                             -0.0,
                             -1.0,
                             -0.04
                     }
             }};
    mropt::freespace::FreeSpace::init_cfree(polygons);
    // 1 - Mission
    int N = 20;
    double T = 10;

    // 1.1) Dubins
//    std::vector<std::vector<double>> start =
//            {
//                    {1, 1, 0.5 * M_PI},
//                    {3, 4, 0.5 * M_PI},
//                    {1, 4, 0}
//            };
//    // 1.2) Goal Configuration (x,y, theta) for 3 robot
//    std::vector<std::vector<double>> goal =
//            {
//                    {2, 2.5, 0},
//                    {3, 2,   0},
//                    {4, 1,   0}
//            };
//    // 1.3) Length and Width
//    std::vector<double> L = {1,
//                             0.6,
//                             1.3};
//    std::vector<double> W = {1, 0.6, 1.3};


//    // 1.1) Unicycle
//    std::vector<std::vector<double>> start =
//            {
//                    {1, 1, -0.5 * M_PI, 0, 0},
//                    {3, 4, 0.5 * M_PI, 0, 0},
//                    {1, 4, 0, 0, 0}
//            };
//    // 1.2) Goal Configuration (x,y, theta) for 3 robot
//    std::vector<std::vector<double>> goal =
//            {
//                    {2, 2.5,  0,0,0 },
//                    {3, 2,   0, 0, 0},
//                    {4, 1, 0, 0,0}
//            };
//    // 1.3) Length and Width
//    std::vector<double> L = {1, 0.6, 1.3};
//    std::vector<double> W = {1, 0.6, 1.3};

    // 1.1) Unicycle
    std::vector<std::vector<double>> start =
            {
                    {1, 1, cos(-0.5 * M_PI), sin(-0.5 * M_PI)},
                    {3, 4, cos(0.5 * M_PI), sin(0.5 * M_PI)},
                    {1, 4,cos(0), sin(0)}
            };
    // 1.2) Goal Configuration (x,y, theta) for 3 robot
    std::vector<std::vector<double>> goal =
            {
                    {2, 2.5, cos(0), sin(0)},
                    {3, 2,  cos(0), sin(0)},
                    {4, 1, cos(0), sin(0)}
            };
    // 1.3) Length and Width
    std::vector<double> L = {1, 0.6, 1.3};
    std::vector<double> W = {1, 0.6, 1.3};



    // Test MPI treads is equal to number of robots
    int R = start.size();
    if (size != R) {
        std::cerr << "Decoupled case must have R mpi thread!" << std::endl;
    }

    // 2 - Build a Robot
//    mropt::Problem::BuilderDistributedRobot_Unicycle builder_unicycle_car(L[r]);
//    auto params = mropt::Problem::Robot::Params{0.0, T, N};
//    builder_unicycle_car.make_robot(r, params);
//    auto robot_d = builder_unicycle_car.getDistributedRobot();
//    mropt::Problem::BuilderDistributedRobot_Dubins builder_dubins_car(L[r]);
//    auto params = mropt::Problem::Robot::Params{0.0, T, N};
//    builder_dubins_car.make_robot(r, params);
//    auto robot_d = builder_dubins_car.getDistributedRobot();
    mropt::Problem::BuilderRobot_DiffDriveQuat builder_ddquat_car(L[r]);
    auto params = mropt::Problem::Robot::Params{0.0, T, N};
    builder_ddquat_car.make_robot(r, params);
    auto robot_d = builder_ddquat_car.getDistributedRobot();
    // 3.2 - Assign missions to the robots
    auto pas = mropt::freespace::FreeSpace::stay_always_in_polygon(N,0);
    robot_d->addMission(start[r], goal[r], {pas});

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

    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> y;
    std::vector<std::vector<double>> o;
    std::vector<std::vector<std::vector<double>>> u;
    robot_d->data_shared->getMRTrajectory(x, y, o, u, robot_d);

    mrprob_d.plot_trajectories(std::vector<std::shared_ptr<mropt::Dynamics::ode>>(R, robot_d->get_ode()));
    MPI_Barrier(MPI_COMM_WORLD);
/*
    } catch (...) {
        std::exit(1); // terminate with exit code 1 = fail
    }
*/



}