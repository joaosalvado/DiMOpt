
//
// Created by ohmy on 2021-07-04.
//

#include "FirstOrderTaylorCollisions.h"

using namespace mropt::collisions;

void FirstOrderTaylorCollisions::convexify(Opti &ocp) {
  for(auto &collision_approx : this->g_col) {
    auto r1 = collision_approx.first.r1;
    auto r2 = collision_approx.first.r2;
    auto k = collision_approx.first.k;
    const auto &xy_k_r1 = X0_num(r1)(xy_slice, k);
    const auto &xy_k_r2 = X0_num(r2)(xy_slice, k);
    DM total_safe_dist = safe_dist(r1) + safe_dist(r2);
    ocp.set_value(// Set f0
        collision_approx.second.fot->f_0,
        col_.f(xy_k_r1, xy_k_r2) + col_.threshold);
    ocp.set_value(// Set jac_f0
        collision_approx.second.fot->jac_f_0,
        col_.jac_f(xy_k_r1, xy_k_r2) );
  }
}

void FirstOrderTaylorCollisions::generate_constraint(
    casadi::Opti& ocp, int r1, int r2, int k){
  Collision col_id{r1, r2, k};
  Approximation approx{};
  approx.fot = std::make_shared<Fot>();
  approx.fot->f_0 = ocp.parameter(1,1);
  approx.fot->jac_f_0 = ocp.parameter(1,4);
  const auto &xy_r1 = MX::sym("xy_r1", 2,1); //symbolic
  const auto &xy_r2 = MX::sym("xy_r2", 2, 1);//symbolic
  const auto &X0_r1 = *X0_sym(r1);
  const auto &X0_r2 = *X0_sym(r2);
  const MX &fo_taylor
    = approx.fot->f_0
        + mtimes( approx.fot->jac_f_0,
                  MX::vertcat({ xy_r1 - X0_r1(xy_slice,k),
                                      xy_r2 - X0_r2(xy_slice,k)}) );
  //const MX &fo_taylor = approx.fot->f_0 + mtimes( approx.fot->jac_f_0, MX::vertcat({ (xy_r1-xy_r2) - (X0_r1(xy_slice,k)-X0_r2(xy_slice,k) )})  );
  approx.f_ap
    = std::make_shared<Function>(
        Function("f_c", {{xy_r1}, {xy_r2}}, {fo_taylor}) );
  g_col.insert({{col_id, approx}});// map [collision] = approx
}