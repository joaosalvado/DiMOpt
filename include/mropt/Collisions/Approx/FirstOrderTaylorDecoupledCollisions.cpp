//
// Created by ohmy on 2021-09-11.
//

#include "FirstOrderTaylorDecoupledCollisions.h"

using namespace mropt::collisions;

FirstOrderTaylorDecoupledCollisions::FirstOrderTaylorDecoupledCollisions(const Collisions &Col)
    : CollisionsAugLagrangian(Col) {}

void FirstOrderTaylorDecoupledCollisions::convexify(int r) {
  for(auto &pcol_entry : this->g_col_param[r]){
    auto &collision = pcol_entry.first;
    auto r1 = collision.r1;
    auto r2 = collision.r2;
    auto k = collision.k;
    auto &pcol = pcol_entry.second;
    const auto &xy_k_r1 = X0_num(r1)(xy_slice, k);
    const auto &xy_k_r2 = pcol.x2_param; // x2 is set as parameter
    DM total_safe_dist
        = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
    robots[r1]->ocp_->set_value(// Set f0
        pcol.approx.fot->f_0,
        col_.f(xy_k_r1, robots[r1]->ocp_->value(xy_k_r2)) + col_.threshold);
    robots[r1]->ocp_->set_value(// Set jac_f0
        pcol.approx.fot->jac_f_0,
        col_.jac_f(xy_k_r1, robots[r1]->ocp_->value(xy_k_r2)) );
  }
}
FirstOrderTaylorDecoupledCollisions::Approximation
FirstOrderTaylorDecoupledCollisions::generate_constraint(Opti &ocp, int r1, int k) {
  Approximation approx{};
  approx.fot = std::make_shared<Fotd>();
  approx.fot->f_0 = ocp.parameter(1,1);
  approx.fot->jac_f_0 = ocp.parameter(1,4);

  const auto &xy_r1 = MX::sym("xy_r1", 2,1); //symbolic
  const auto &X0_r1 = *X0_sym(r1);
  const MX &fo_taylor
      = approx.fot->f_0
          + mtimes( approx.fot->jac_f_0,
                    MX::vertcat({ xy_r1 - X0_r1(xy_slice,k),
                                  DM::zeros(2,1)}) );
  approx.f_ap
      = std::make_shared<Function>(
      Function("f_c", {{xy_r1}}, {fo_taylor}) );
  return approx;
}
