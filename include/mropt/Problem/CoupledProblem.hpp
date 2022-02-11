#ifndef OPTPROBLEM_H
#define OPTPROBLEM_H
#pragma once

#include <mropt/util/Plotter.hpp>
#include "casadi/casadi.hpp"
#include "Robot.hpp"
#include "mropt/Collisions/CollisionsApprox.h"
#include "mropt/Collisions/CollisionsAugLagrangian.h"
#include <future>

#include "mropt/util/Recorder.h"

namespace mropt::Problem {
class CoupledProblem {
private:
  std::vector<std::shared_ptr<Robot>> robots;
  std::shared_ptr<mropt::collisions::CollisionsApprox> collisions_c;
  // --- Problem and Solver Options
  Dict p_opts, s_opts;
  std::string solver{"ipopt"};
  std::shared_ptr<Opti> ocp;
  std::vector<std::shared_ptr<Opti>> ocps;
  //Trajectory Plotter
  std::shared_ptr<mropt::util::Plotter> plotter;

  mropt::util::Recorder::Record record_curr;

public:
  explicit CoupledProblem() {
    solution = nullptr;
    plotter = nullptr;
  }

  CoupledProblem &addRobot(const std::shared_ptr<Robot> &robot);
  CoupledProblem &addCollisionsCoupled(
    std::shared_ptr<mropt::collisions::CollisionsApprox> collisions_approx) {
    collisions_c = std::move(collisions_approx);
    return *this;
  }
  CoupledProblem &addRobots(const std::vector<std::shared_ptr<Robot>> &robots_);
  void solve();

  DM actual_cost();
  DM prev_actual_cost();
  // bool handle_solution_decoupled(const std::vector<double> &pho, double pho_prime);
  bool handle_solution_coupled(const std::vector<double> &pho, double pho_prime);
  void compute_robot_costs_violations(
      std::vector<double> &pho,
      OptiSol &solution);
  void compute_collisions_costs_violations(
      OptiSol &solution);
  bool backtrack(DM &true_cost);
  bool viol_improv(double tol);

  CoupledProblem &allow_plotting();
  CoupledProblem &debug_mode();

  virtual ~CoupledProblem() = default;

  mropt::util::Recorder::Record& get_record(){ return this->record_curr;}

  //Plotting
  CoupledProblem &show_stats();
  CoupledProblem &set_plotter(const std::shared_ptr<mropt::util::Plotter> &plotter) {
    this->plotter = plotter;
    return *this;
  }
  CoupledProblem &plot_trajectories();

  void reset_ocp();
  double get_cost(){ return this->actual_cost().scalar();}
  bool solved_successfully{true};
private:
  bool plot_{false}; bool debug_{false};
  //[COUPLED] SCP params
  double x_tol{1.0e-1};
  double g_tol{1.0e-2};
  double f_tol{1.0e-1};
  double k = 10;
  double pho0{0.0}; // reject and shrink
  double pho1{0.1}; // accept but shrink
  //double pho2{0.75}; // accept and expand
  DM delta_f{10.0};
  DM delta_x{10.0};
  //Costs
  DM total_prev_true_cost, total_true_cost, total_model_cost;
  //Violation checker
  DM prev_violation{-10};
  bool failed;
    bool iter1 = true; //TODO: testing

  //Time vars
  //duration<double, std::milli> ocp_create_time{0};
  duration<double, std::milli> ocp_query_time{0};

  std::shared_ptr<OptiSol> solution;



  //Functions
  void reset();
  void setup();
  void convexify();
  void warm_start();
  void scp();
};
}
#endif