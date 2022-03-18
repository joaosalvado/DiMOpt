//
// Created by ohmy on 2021-09-15.
//

#include <casadi/casadi.hpp>
#include <cmath>
#include <string>

#include "mropt/StateSpace/SE2/SE2.hpp"
#include "mropt/ControlSpace/R2/VW.hpp"
#include "mropt/Dynamics/CarLike/DiffDrive.hpp"
#include "mropt/Dynamics/Approx/FirstOrderTaylor.hpp"
#include "mropt/Dynamics/Transcription/LocalCollocation/MultipleShootingApprox.hpp"
#include "mropt/FreeSpace/FreeSpace.hpp"
#include "mropt/Cost/Cost.hpp"
#include "mropt/Collisions/L2norm/L2NormCollision.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorCollisions.h"
#include "mropt/Problem/Robot.hpp"
#include "mropt/Problem/CoupledProblem.hpp"
#include "mropt/Problem/DecoupledProblem.hpp"
#include "mropt/RobotShape/CircleRobot.h"
#include "mropt/util/Opencv_plotter.hpp"
#include "mropt/Collisions/Approx/FirstOrderTaylorDecoupledCollisions.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorDistributedCollisions.h"
#include "mropt/Problem/DistributedRobot.h"


#include <mpi.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "mropt/util/Recorder.h"


void init_cfree(std::vector<std::vector<std::vector<double>>> polygons);


int main(int argc, char** argv) {
    // Initialisation
    MPI_Init( &argc, &argv );
    // Reading size and rank
    int r, size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    MPI_Comm_rank(MPI_COMM_WORLD, &r);

    // -1 - Read Mission
    std::string maps_path = "../maps/";
    std::string record_file = "../records/distributed.txt";
    // 0 - Get Mission data from file
    std::string mission_filename {argv[1]};
    std::string map_file {argv[2]};
    std::ifstream mission_filestream(mission_filename);
    json mission_json;
    mission_filestream >> mission_json;
    auto mission_id = mission_json["mission_id"].get<int>();
    std::cout << "[SCP] Start Solving Mission " << mission_id << std::endl;
    auto N = mission_json["N"].get<int>();
    auto T = mission_json["T"].get<double>();
    auto start = mission_json["start"].get<std::vector<std::vector<double>>>();
    auto goal = mission_json["goal"].get<std::vector<std::vector<double>>>();
    auto L = mission_json["L"].get<std::vector<double>>();
    auto W = mission_json["W"].get<std::vector<double>>();
    auto polygons = mission_json["polygons"].get<std::vector<std::vector<std::vector<double>>>>();
    auto assignments = mission_json["assignments"].get<std::vector<std::vector<std::vector<int>>>>();

    // Decoupled case threads test
    int R = start.size();
    if(size != R){
        std::cerr << "Decoupled case must have R mpi thread!" << std::endl;
    }

    // 1 - Initialize robots
    //State Space
    auto se2 = std::make_shared<mropt::StateSpace::SE2>();
    //Control Space
    auto vw = std::make_shared<mropt::ControlSpace::VW>();
    // Cost
    auto cost = std::make_shared<mropt::cost::Cost>(*vw, *se2);
    //Robot Shape
    auto shape = std::make_shared<mropt::RobotShape::CircleRobot>(L[r] / 2);
    // Dynamic Model
    auto model = std::make_shared<mropt::Dynamics::CarLike::DiffDrive>(vw, se2, shape);
    // Model Approx
    auto fot = std::make_shared<mropt::Dynamics::Approx::FirstOrderTaylor>(model);
    // Transcription Approx
    auto ms_approx = std::make_shared<mropt::Dynamics::MultipleShootingApprox>(fot, cost);
    // Free Space
    auto cfree = std::make_shared<mropt::freespace::FreeSpace>(*se2);
    // Collisions Approximation - Coupled
    auto collision_type = mropt::collisions::L2NormCollision();
    // Collisions Augmented Lagrangian - ADMM
    auto dis_col = std::make_shared<mropt::collisions::FirstOrderTaylorDistributedCollisions>(collision_type);
    // Robot
    auto params = mropt::Problem::Robot::Params{0.0, T, N};
    auto robot_d = std::make_shared<mropt::Problem::DistributedRobot>(r, params, vw, se2, cost, cfree, ms_approx, shape, dis_col);

    // 2 - Initialize Free Space
    init_cfree(polygons);

    // 3 - Add Missions to robots
    // 3.1 - Get assignments in correct form
    std::vector<mropt::freespace::FreeSpace::PolygonAssignment> pas_robot;
    for(const auto &as : assignments[r]){
        auto k0 = as[0]; auto kf = as[1]; auto p_id = as[2];
        mropt::freespace::FreeSpace::PolygonAssignment pa{k0, kf, p_id};
        pas_robot.emplace_back(pa);
    }
    // 3.2 - Assign missions to the robots
    robot_d->addMission(start[r], goal[r], pas_robot);

    // 4 -Solve
    // 4.1 - Setup Decoupled Solver
    auto plotter = std::make_shared<mropt::util::Opencv_plotter>(R); // dummy
    plotter->setFootprint({L}, {W});
    plotter->addPathToScenarios(maps_path);
    plotter->inputScenario(map_file); // TODO: remove me only for debug mode

    mropt::Problem::DecoupledProblem mrprob_d{r};
    mrprob_d.setParams(R, N);
    mrprob_d.addRobot(robot_d);
    mrprob_d.set_plotter(plotter);
    mrprob_d.debug_mode().allow_plotting();
    // 4.2 - Solve
    try {
        mrprob_d.solve();
        mrprob_d.plot_trajectories(std::vector<std::shared_ptr<mropt::Dynamics::ode>>(R, robot_d->get_ode()) );
        MPI_Barrier(MPI_COMM_WORLD);
    } catch(...){
        mropt::util::Recorder recorder;
        mropt::util::Recorder::Record record;
        recorder.addRecord(
                record_file,
                record.dummy_failed_record() );
        std::exit(1); // terminate with exit code 1 = fail
    }

    // 5 - Save solution in file
    // 5.0 - Experiments Records
    if(r==0) {
        auto record = mrprob_d.get_record();
        record.A = 10 * 10 * 0.5;
        if(record.fail_status != -2) record.fail_status = 1;
        mropt::util::Recorder recorder;
        recorder.addRecord(record_file, record);
    }
    // 5.1 - gather cost and trajectory
    auto cost_sol = robot_d->data_shared->actual_cost(robot_d);
    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> y;
    std::vector<std::vector<double>> o;
    std::vector<std::vector<std::vector<double>>> u;
    robot_d->data_shared->getMRTrajectory(x, y, o, u, robot_d);
    // 5.2 - save it in json file
    if(r==0) { // save data once, think about it as robot 0 being the master robot
        json result_json;
        result_json["mission_id"] = mission_id;
        result_json["cost"] = cost_sol;
        result_json["x"] = x;
        result_json["y"] = y;
        result_json["o"] = o;
        result_json["u"] = u;

        std::stringstream result;
        result << "result_" << mission_id;
        std::ofstream of(result.str(), std::ofstream::out);
        of << std::setw(4) << result_json << std::endl;
    }

    mropt::freespace::FreeSpace::clear_polygons();
    if(! mrprob_d.solved_successfully) std::exit(1);  // terminate with exit code 1 = fail

    // Finalization
    MPI_Finalize();
}




void init_cfree(std::vector<std::vector<std::vector<double>>> polygons){
    Slice all;
    for(auto &polygon : polygons){
        int Hp = polygon.size(); // amount of halfplanes in polygon
        MX A = MX(Hp, 2);
        MX b = MX(Hp, 1);
        for(int hp_id = 0; hp_id < Hp; ++hp_id){
            auto halfplane = polygon[hp_id];
            auto a1 = halfplane[0];
            auto a2 = halfplane[1];
            auto b1 = halfplane[2];
            A(hp_id, all) = DM({a1, a2});
            b(hp_id) = DM({b1});
        }
        mropt::freespace::FreeSpace::Polygon P{A, b};
        mropt::freespace::FreeSpace::add({P});
    }
}


