#ifndef DECOUPLEDPROBLEM_H
#define DECOUPLEDPROBLEM_H
#pragma once

#include <mropt/util/Plotter.hpp>
#include "casadi/casadi.hpp"
#include "DistributedRobot.h"
#include "mropt/Collisions/CollisionsApprox.h"
#include "mropt/Collisions/CollisionsAugLagrangian.h"
#include "mpi.h"

#include "SharedData.h"
#include "mropt/util/Recorder.h"

namespace mropt::Problem {
class DecoupledProblem {
private:
  int R; // #robots
  int N; // #discrete points
  std::shared_ptr<DistributedRobot> robot;
  std::shared_ptr<mropt::collisions::CollisionsAugLagrangian> collisions_d;
  // --- Problem and Solver Options
  Dict p_opts, s_opts;
  std::string solver{"ipopt"};
  std::shared_ptr<Opti> ocp;
  // Trajectory Plotter
  std::shared_ptr<mropt::util::Plotter> plotter;
  int proc_id_;
  Slice all;
  mropt::util::Recorder::Record record_curr;
public:
  explicit DecoupledProblem(int proc_id)
      : proc_id_(proc_id) {
    solution = nullptr;
    plotter = nullptr;
  }

  DM actual_cost();
  DM prev_actual_cost();

  void setParams(int R_, int N_) {
    R = R_;
    N = N_;
  }

  DecoupledProblem &addRobot(const std::shared_ptr<DistributedRobot> &robot);

  DecoupledProblem &addCollisionsDeCoupled(std::shared_ptr<mropt::collisions::CollisionsAugLagrangian> collisions_al) {
    collisions_d = std::move(collisions_al);
    return *this;
  }

  void solve();

  DecoupledProblem &allow_plotting();
  DecoupledProblem &debug_mode();

  virtual ~DecoupledProblem() = default;

  //Plotting
  DecoupledProblem &show_stats();
  DecoupledProblem &set_plotter(const std::shared_ptr<mropt::util::Plotter> &plotter) {
    this->plotter = plotter;
    return *this;
  }
  DecoupledProblem &plot_trajectories(
      const std::vector<std::shared_ptr<mropt::Dynamics::ode>> &odes);

  void reset_ocp();
  bool solved_successfully{true};

  util::Recorder::Record& get_record(){ return this->record_curr;}
private:
  bool plot_{false};
  bool debug_{false};

  //Costs
  double total_prev_true_cost, total_true_cost, total_model_cost;

  //Time vars
  duration<double, std::milli> ocp_query_time{0};

  std::shared_ptr<OptiSol> solution;

  //Functions
  void reset();
  void setup();
  void warm_start();
  void admm();
  void test();
};
}
#endif