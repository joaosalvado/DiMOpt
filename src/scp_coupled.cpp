//
// Created by ohmy on 2021-09-15.
//

#include <casadi/casadi.hpp>
#include <string>
#include <nlohmann/json.hpp>
#include "mropt/FreeSpace/FreeSpace.hpp"

#include "mropt/StateSpace/SE2/SE2.hpp"
#include "mropt/ControlSpace/R2/VW.hpp"
#include "mropt/Dynamics/CarLike/DiffDrive.hpp"
#include "mropt/Dynamics/Approx/FirstOrderTaylor.hpp"
#include "mropt/Dynamics/Transcription/MultipleShooting.hpp"
#include "mropt/Dynamics/Transcription/MultipleShootingApprox.hpp"

#include "mropt/Cost/Cost.hpp"
#include "mropt/Collisions/L2norm/L2NormCollision.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorCollisions.h"
#include "mropt/Problem/Robot.hpp"
#include "mropt/Problem/CoupledProblem.hpp"
#include "mropt/Problem/DecoupledProblem.hpp"
#include "mropt/RobotShape/CircleRobot.h"

#include "mropt/util/Opencv_plotter.hpp"


#include <mpi.h>


using json = nlohmann::json;

void init_cfree(std::vector<std::vector<std::vector<double>>> polygons);


int main(int argc, char** argv) {
    bool debug_ = true;
    // Initialisation
    MPI_Init( &argc, &argv );
    // Reading size and rank
    int rank, size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    // Coupled case test
    if(rank != 0){
        std::cerr << "Coupled case must only have one mpi thread!" << std::endl;
    }

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

    // 0 - Generate Cfree
    int R = start.size();
    init_cfree(polygons);

    // 1 - Create Robots
    std::vector<std::shared_ptr<mropt::Problem::Robot>> robots;
    for(int r = 0; r < R; ++r){
        // 1 - Initialize Robots:
        // 1.1 - State Space
        auto se2 = std::make_shared<mropt::StateSpace::SE2>();
        // 1.2 - Control Space
        auto vw = std::make_shared<mropt::ControlSpace::VW>();
        // Robot Shape
        auto shape = std::make_shared<mropt::RobotShape::CircleRobot>(L[r] / 2);
        // 1.3 - Dynamic Model
        auto model = std::make_shared<mropt::Dynamics::CarLike::DiffDrive>(vw, se2, shape);
        // Model Approx
        auto fot = std::make_shared<mropt::Dynamics::Approx::FirstOrderTaylor>(model);
        // Transcription Approx
        auto ms_approx = std::make_shared<mropt::Dynamics::MultipleShootingApprox>(fot);
        // Cost
        auto cost = std::make_shared<mropt::cost::Cost>(*vw, *se2);
        // Free Space
        auto cfree_mropt = std::make_shared<mropt::freespace::FreeSpace>(*se2);
        // Create Robot
        auto params = mropt::Problem::Robot::Params{0.0, T, N};
        auto robot = std::make_shared<mropt::Problem::Robot>(params, vw, se2, cost, cfree_mropt, ms_approx, shape);
        robots.push_back(robot); // add it
    }

    // 2 - Assign missions
    // 2.1 - Get assignments in correct form
    std::vector<std::vector<mropt::freespace::FreeSpace::PolygonAssignment>> pas;
    for(const auto &robot : assignments){
        std::vector<mropt::freespace::FreeSpace::PolygonAssignment> pas_robot;
        for(const auto &as : robot){
            auto k0 = as[0]; auto kf = as[1]; auto p_id = as[2];
            mropt::freespace::FreeSpace::PolygonAssignment pa{k0, kf, p_id};
            pas_robot.emplace_back(pa);
        }
        pas.push_back(pas_robot);
    }
    // 2.2 - Assign missions to the robots
    for( int r = 0; r < R; ++r){
        robots[r]->addMission(start[r], goal[r], {pas[r]});
    }

    // 3 - Collisions Approximation - Coupled
    auto collision_type = mropt::collisions::L2NormCollision();
    auto fot_col = std::make_shared<mropt::collisions::FirstOrderTaylorCollisions>(collision_type);
    // 4 - Overall Problem
    mropt::Problem::CoupledProblem mrprob{};

    if(debug_) {
        // Plotting
//  //TODO : debugging remove me
        std::string map_file = "0.5pol.png";
        std::string maps_path = "/home/ohmy/js_ws/github_repos/mrrm/maps/";
        auto plotter = std::make_shared<mropt::util::Opencv_plotter>(R);
        plotter->setFootprint(L, W);
        plotter->addPathToScenarios(maps_path);
        plotter->inputScenario(map_file);
        if(debug_) mrprob.set_plotter(plotter).allow_plotting();
    }


    if(R > 1){
        mrprob.addRobots(robots).addCollisionsCoupled(fot_col).allow_plotting();
    } else{ // Single Robot
        mrprob.addRobots(robots).allow_plotting();
    }
    //  5 - Solve it
    bool success {true};
    try {
        mrprob.solve();
    } catch(...){
        mropt::util::Recorder recorder;
        mropt::util::Recorder::Record record;
        recorder.addRecord(
                record_file,
                record.dummy_failed_record() );
        std::exit(1); // terminate with exit code 1 = fail
    }
    //mrprob.show_stats().plot_trajectories();

    // 6 - Save solution in file
    // 6.0 - Save experiments record
    auto record = mrprob.get_record();
    record.A = 10 * 10 * 0.5;
    record.fail_status = 1;
    mropt::util::Recorder recorder;
    recorder.addRecord(record_file, record);
    // 6.1 - Save the Solution (JSON)
    std::vector<std::vector<double>> x, y, o;
    std::vector<std::vector<std::vector<double>>> u;
    for(int r = 0; r < R; ++r){
        std::vector<double> x_r, y_r, o_r; std::vector<std::vector<double>> u_r;
        robots[r]->get_solution(x_r,y_r,o_r, u_r);
        x.push_back(x_r); y.push_back(y_r); o.push_back(o_r); u.push_back(u_r);
    }
    auto cost_sol = mrprob.get_cost();
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


    mropt::freespace::FreeSpace::clear_polygons();
    if(! mrprob.solved_successfully) std::exit(1);  // terminate with exit code 1 = fail
    // Finalisation
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
