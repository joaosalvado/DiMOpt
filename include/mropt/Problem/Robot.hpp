#ifndef ROBOT_H
#define ROBOT_H
#pragma once

#include "../StateSpace/State.hpp"
#include "../ControlSpace/Control.hpp"
#include "../Cost/Cost.hpp"
#include "../Dynamics/Transcription.hpp"
#include "../FreeSpace/FreeSpace.hpp"
#include "../RobotShape/Footprint.h"
#include "SingleMission.hpp"

#include "casadi/casadi.hpp"
#include <matplotlibcpp.h>
#include "../util/integration.hpp"

// Forward class declare
namespace mropt::collisions{
 class CollisionsApprox;
 class CollisionsAugLagrangian;
 class DistributedCollisions;
 class FirstOrderTaylorDecoupledCollisions;
 class FirstOrderTaylorDistributedCollisions;
}

namespace plt = matplotlibcpp;

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace mropt::Problem {
class Robot {
public:

  struct Trajectory0 {
    std::shared_ptr<MX> U0_;
    std::shared_ptr<MX> X0_;
  };
  struct Params {
    double t0;
    double tf;
    int N;
  };

  Robot(const Robot::Params &P,
        const std::shared_ptr<mropt::ControlSpace::Control> &Cs,
        const std::shared_ptr<mropt::StateSpace::State> &Ss,
        const std::shared_ptr<mropt::cost::Cost> &Cost,
        const std::shared_ptr<mropt::freespace::FreeSpace> &Fspace,
        const std::shared_ptr<mropt::Dynamics::Transcription> &Dynamics,
        const std::shared_ptr<mropt::RobotShape::Footprint> &Shape);

  Robot(const Robot &robot);
  Robot &operator=(const Robot &robot) = delete;

  virtual ~Robot() = default;

  bool backtrack();

  void addMission(
      const std::vector<double> &x_init,
      const std::vector<double> &x_goal,
      const std::vector<mropt::freespace::FreeSpace::PolygonAssignment> &pas) {
    SingleMission mission(*ss);
    mission.init(x_init);
    mission.goal(x_goal);
    mission.path(pas);
    missions.push_back(std::move(mission));
  };

  void resetQuery(Opti &ocp);
  void setupQuery(Opti &ocp);
  void scp();
  DM true_cost_l1penalty(const DM &X, const DM &U);
  void plot();
  void initial_guess(SingleMission &mission);
  void convexify_dynamics(Opti &ocp, DM &x0, DM &u0) {
    ocp.set_value(*(traj0.U0_), u0);
    ocp.set_value(*(traj0.X0_), x0);
    dynamics->convexify(ocp, x0, u0);
  }
  void get_solution(
      std::vector<double> &x,
      std::vector<double> &y,
      std::vector<double> &o,
      std::vector<std::vector<double>> &u) {
    x = x_sol;
    y = y_sol;
    o = o_sol;
    u = u_sol;
  }
  Params p_;

protected:
protected:
  friend class CoupledProblem;
  friend class DecoupledProblem;
  friend class SharedData;
  friend class mropt::collisions::CollisionsApprox;
  friend class mropt::collisions::CollisionsAugLagrangian;
  friend class mropt::collisions::DistributedCollisions;
  friend class mropt::collisions::FirstOrderTaylorDecoupledCollisions;
  friend class mropt::collisions::FirstOrderTaylorDistributedCollisions;


  std::shared_ptr<mropt::ControlSpace::Control> cs;
  std::shared_ptr<mropt::StateSpace::State> ss;
  std::shared_ptr<mropt::cost::Cost> cost;
  std::shared_ptr<mropt::freespace::FreeSpace> fspace;
  std::shared_ptr<mropt::Dynamics::Transcription> dynamics;
  std::shared_ptr<mropt::RobotShape::Footprint> shape;
  bool plot_{false};
  bool debug_{false};
  // Missions
  std::deque<SingleMission> missions;
  std::shared_ptr<SingleMission> mission_curr;
  // ocp
  std::shared_ptr<Opti> ocp_;
  Trajectory0 traj0;
  DM X_curr, U_curr;
  DM X_sol, U_sol;
  // Free Space
  MX mu_free;
  DM mu_f_0{10};
  const DM mu_f_0_init{10};
  MX sum_g_free{0.0};
  MX max_g_free{0.0};
  // Dynamics
  MX mu_dynamics;
  DM mu_d_0{10};
  const DM mu_d_0_init{10};
  MX sum_g_dynamics{0.0};
  MX max_g_dynamics{0.0};
  // Collisions Decoupled
  MX mu_col;
  DM mu_c_0{10};
  const DM mu_c_0_init{10};
  MX sum_g_col{0.0};
  MX max_g_col{0.0};
  Function col_J_viol_;
  Function col_J_real_fast_;
  MX col_J_real_;

  MX x2param{};
  std::function<void()> convexify_decoupled_collisions;
  MX rho;
  // Model Cost
  MX J_model{0.0}, J_model_nocol;
  DM true_cost{0.0}, prev_true_cost{0.0}, model_cost{0.0};
  DM dynamics_violation{0.0}, free_space_violation{0.0}, collisions_violation{0.0};
  // True Cost Function
  std::function<DM(const DM &X, const DM &U)> true_cost_;
  Slice all;

  // Auxiliary Plot vars
  std::vector<double> true_cost_vec{};
  std::vector<double> model_cost_vec{};
  std::vector<double> dynamics_violation_vec{};
  std::vector<double> free_space_violation_vec{};
  std::vector<double> collisions_violation_vec{};
  std::vector<double> x_sol, y_sol, o_sol;
  std::vector<std::vector<double>> u_sol;
  std::vector<std::vector<double>> x_sol_i, y_sol_i, o_sol_i;
  void clear_plot_vars();

  //Scp vars
  double x_tol{1.0e-1};
  double g_tol{1.0e-2};
  double f_tol{1.0e-1};
  double k = 10;
  double pho0{0.0}; // reject and shrink
  double pho1{0.1}; // accept but shrink
  //double pho2{0.75}; // accept and expand
  DM delta_f{10.0};
  DM delta_x{10.0};
  bool failed;
  //Time vars
  //duration<double, std::milli> ocp_create_time{0};
  duration<double, std::milli> ocp_query_time{0};
  //Costs
  //Violation checker
  DM prev_violation{-10};
  DM prev_violation_dynamics{-10};
  DM prev_violation_freespace{-10};
  DM prev_violation_collisions{-10};
  //solution store
  std::shared_ptr<OptiSol> solution;
  //Scp help functions
  double compute_robot_costs_violations();
  bool handle_solution(double pho);
  bool viol_improv(double tol);

  double UniformNoise(double start, double end);

};
}
#endif




