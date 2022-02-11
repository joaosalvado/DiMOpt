#include <casadi/casadi.hpp>
#include <cmath>
#include <matplotlibcpp.h>
#include <string>

#include "mropt/StateSpace/SE2/SE2.hpp"
#include "mropt/ControlSpace/R2/VW.hpp"
#include "mropt/Dynamics/CarLike/DiffDrive.hpp"
#include "mropt/Dynamics/Approx/FirstOrderTaylor.hpp"
#include "mropt/Dynamics/Transcription/MultipleShooting.hpp"
#include "mropt/Dynamics/Transcription/MultipleShootingApprox.hpp"
#include "mropt/FreeSpace/FreeSpace.hpp"
#include "mropt/Cost/Cost.hpp"
#include "mropt/Collisions/L2norm/L2NormCollision.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorCollisions.h"
#include "mropt/Problem/Robot.hpp"
#include "mropt/Problem/CoupledProblem.hpp"
#include "mropt/RobotShape/CircleRobot.h"

#include "mropt/util/Opencv_plotter.hpp"

#include "mropt/Collisions/CollisionsAugLagrangian.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorDecoupledCollisions.h"

void inverted_c_cfree(){
  // Constructing Cfree set of polygons
  //Normals Rectangle
  MX A = MX(4, 2);
  A = DM(
      {{-1, 0},
       {0, 1},
       {1, 0},
       {0, -1}});
  FreeSpace::Polygon P1{A, DM::vertcat({0, 3, 10, 0})};
  FreeSpace::Polygon P2{A, DM::vertcat({-7, 10, 10, 0})};
  FreeSpace::Polygon P3{A, DM::vertcat({0, 10, 10, -7})};
  FreeSpace::add({P1, P2, P3});
  FreeSpace::poly_centers =
      new std::unordered_map<std::pair<int, int>, State::state, boost::hash<std::pair<int, int>>>
          {
              {{P1.id_, P2.id_}, {8.5, 1.5, M_PI / 2}},
              {{P2.id_, P1.id_}, {8.5, 1.5, M_PI}},
              {{P2.id_, P3.id_}, {8.5, 8.5, M_PI}},
              {{P3.id_, P2.id_}, {8.5, 8.5, -M_PI / 2}}
          };
}




int main() {
  std::string map_file = "c_map.png";
  std::string maps_path = "/home/ohmy/js_ws/github_joao/MROpt/maps/";

  int R = 3;
  int N = 50;
  double T = 50.0;
  //State Space
  auto se2 = std::make_shared<SE2>();
  //Control Space
  auto vw = std::make_shared<VW>();
  // Dynamic Model
  double L = 0.7;
  double L3 = L, L2 =L;
  auto model = std::make_shared<DiffDrive>(vw, se2, L);
  //Robot Shape
  auto shape = std::make_shared<CircleRobot>(L / 2);
  // Model Approx
  auto fot = std::make_shared<FirstOrderTaylor>(model);
  // Transcription Approx
  auto ms_approx = std::make_shared<MultipleShootingApprox>(fot);
  // Cost
  auto cost = std::make_shared<Cost>(*vw, *se2);
  // Free Space
  auto cfree = std::make_shared<FreeSpace>(*se2);
  inverted_c_cfree(); //TODO: by hand now, use mrenv
  // Collisions Approximation - Coupled
  auto collision_type = L2NormCollision();
  auto fot_col = std::make_shared<FirstOrderTaylorCollisions>(collision_type);

  // Collisions Augmented Lagrangian - ADMM
  auto lag_col = std::make_shared<FirstOrderTaylorDecoupledCollisions>(collision_type);

  // Problem
  auto params = Robot::Params{0.0, T, N};
  auto robot1 = std::make_shared<Robot>(params, vw, se2, cost, cfree, ms_approx, shape);
//  auto robot2 = std::make_shared<Robot>(*robot1);//copy
//  auto robot3 = std::make_shared<Robot>(*robot1);//copy
  // Robot 2
  auto se2_r2 = std::make_shared<SE2>(); auto vw_r2 = std::make_shared<VW>();
  auto model_r2 =  std::make_shared<DiffDrive>(vw_r2, se2_r2, L2);
  auto shape_r2 = std::make_shared<CircleRobot>(L2/2);
  auto fot_r2 = std::make_shared<FirstOrderTaylor>(model_r2);
  auto ms_approx_r2 = std::make_shared<MultipleShootingApprox>(fot_r2);
  auto cost_r2 = std::make_shared<Cost>(*vw_r2, *se2_r2);
  auto cfree_r2 = std::make_shared<FreeSpace>(*se2_r2);
  auto robot2 = std::make_shared<Robot>(params, vw_r2, se2_r2, cost_r2, cfree_r2, ms_approx_r2, shape_r2);
  // Robot 3
  auto se2_r3 = std::make_shared<SE2>(); auto vw_r3 = std::make_shared<VW>();;
  auto model_r3 =  std::make_shared<DiffDrive>(vw_r3, se2_r3, L3);
  auto shape_r3 = std::make_shared<CircleRobot>(L3/2);
  auto fot_r3 = std::make_shared<FirstOrderTaylor>(model_r3);
  auto ms_approx_r3 = std::make_shared<MultipleShootingApprox>(fot_r3);
  auto cost_r3 = std::make_shared<Cost>(*vw_r3, *se2_r3);
  auto cfree_r3 = std::make_shared<FreeSpace>(*se2_r3);
  auto robot3 = std::make_shared<Robot>(params, vw_r3, se2_r3, cost_r3, cfree_r3, ms_approx_r3, shape_r3);



  // Set Missions: x0, xf and polygon assignments
  FreeSpace::PolygonAssignment p1a{0, (int) N / 3 + 1, 0};
  FreeSpace::PolygonAssignment p2a{(int) N / 3, (int) 2 * N / 3 + 1, 1};
  FreeSpace::PolygonAssignment p3a{(int) 2 * N / 3, N, 2};
  FreeSpace::PolygonAssignment pr2{0, N, 2};
  FreeSpace::PolygonAssignment pr3{0, N, 1};
  double x_0{1.0}, y_0{2.0}, o_0{M_PI / 2};
  double x_f{1.0}, y_f{8.0}, o_f{M_PI / 2};
////  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a, p2a, p3a});
////  robot2->addMission({8.0, y_f, o_0}, {x_f+3.0, y_f}, {pr2});
//  FreeSpace::PolygonAssignment p1b{0, (int) N / 3 + 1, 2};
//  FreeSpace::PolygonAssignment p2b{(int) N / 3, (int) 2 * N / 3 + 1, 1};
//  FreeSpace::PolygonAssignment p3b{(int) 2 * N / 3, N, 0};
  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a, p2a, p3a});
  robot2->addMission({x_f+2.0, y_f, o_0}, {8.0, y_f}, {pr2});
  robot3->addMission({8.0, 1.0, o_0}, {y_f, y_f+1}, {pr3});
//  FreeSpace::PolygonAssignment p1a{0, (int) N , 0};
//  double x_0{2.0}, y_0{1.0}, o_0{0};
//  double x_f{3.0}, y_f{2.0}, o_f{0};
//  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a});
//  robot2->addMission({x_0, y_f, o_f}, {x_f, y_0}, {p1a});

  // Plotting Setup
  auto plotter = std::make_shared<Opencv_plotter>(R);
  plotter->setFootprint({L, L, L}, {0, 0, 0});
  plotter->addPathToScenarios(maps_path);
  plotter->inputScenario(map_file);

  // Solving Decoupled
  OptProblem mrprob_d("decoupled");
  mrprob_d.set_plotter(plotter).allow_plotting();
  mrprob_d.addRobots( {robot1, robot2, robot3} ).addCollisionsDeCoupled(lag_col);
  mrprob_d.solve();
  mrprob_d.show_stats().plot_trajectories();

  // Re add Missions
//  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a, p2a, p3a});
//  robot2->addMission({8.0, y_f, o_0}, {x_f+3.0, y_f}, {pr2});

//  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a, p2a, p3a});
//  robot2->addMission({x_f, y_f, o_0}, {8.0, y_f}, {pr2});

  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a, p2a, p3a});
  robot2->addMission({x_f+2.0, y_f, o_0}, {8.0, y_f}, {pr2});
  //robot3->addMission({x_f, y_f, o_0}, {y_f, y_f}, {p1b, p2b});
  robot3->addMission({8.0, 1.0, o_0}, {y_f, y_f+1}, {pr3});
//  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a});
//  robot2->addMission({x_0, y_f, o_f}, {x_f, y_0}, {p1a});
  // Solving Coupled
  OptProblem mrprob("coupled");
  mrprob.set_plotter(plotter).allow_plotting();
  mrprob.addRobots( {robot1, robot2, robot3} ).addCollisionsCoupled(fot_col);
  mrprob.solve();
  mrprob.show_stats().plot_trajectories();

}


//  //Multiple robots
//  double L3 = 0.4;
//  auto se2_r3 = se2_r1; auto vw_r3 = vw_r1;
//  auto model_r3 = DiffDrive(vw_r3, se2_r3, L3);
//  auto shape_r3 = std::make_shared<CircleRobot>(L3/2);
//  auto fot_r3 = FirstOrderTaylor(model_r3);
//  auto ms_approx_r3 = MultipleShootingApprox(fot_r3);
//  auto cost_r3 = Cost(vw_r3, se2_r3);
//  auto cfree_r3 = FreeSpace(se2_r3);
//  auto robot3 = std::make_shared<Robot>(vw_r3, se2_r3, cost_r3, cfree_r3, ms_approx_r3, shape_r3, params);
//  robot3->addMission({3.0, 1.0, M_PI/2}, {3.0, 8.0}, {pmr1a, pmr2a, pmr3a });