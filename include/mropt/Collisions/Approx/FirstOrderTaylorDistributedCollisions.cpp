//
// Created by ohmy on 2021-09-19.
//

#include "FirstOrderTaylorDistributedCollisions.h"

using namespace mropt::collisions;

FirstOrderTaylorDistributedCollisions::FirstOrderTaylorDistributedCollisions(const Collisions &Col)
    : DistributedCollisions(Col) {}

void FirstOrderTaylorDistributedCollisions::convexify() {
  for(auto &pcol_entry : this->g_col_param){
    auto &collision = pcol_entry.first;
    auto r2 = collision.r2;
    auto k = collision.k;
    auto &pcol = pcol_entry.second;
    const auto &xy_k_r1 = robot->X_curr(xy_slice, k);
    const auto &xy_k_r2 = pcol.x2_param; // x2 is set as parameter
    /*robot->ocp_->set_value(// Set f0
        pcol.approx.fot->f_0,
        col_.f(xy_k_r1, data_shared->Xcurr(r2,k)) + col_.threshold);
    //robot->ocp_->value(xy_k_r2)
    robot->ocp_->set_value(// Set jac_f0
        pcol.approx.fot->jac_f_0,
        col_.jac_f(xy_k_r1, data_shared->Xcurr(r2,k)) );*/
      const auto &Zr2k = data_shared->Z_val(r2,k);
      robot->ocp_->set_value(// Set f0
              pcol.approx.fot->f_0,
              col_.f(xy_k_r1,  Zr2k) ); //TODO: + col_.threshold
      //robot->ocp_->value(xy_k_r2)
      robot->ocp_->set_value(// Set jac_f0
              pcol.approx.fot->jac_f_0,
              col_.jac_f(xy_k_r1, Zr2k ));
  }
}
FirstOrderTaylorDistributedCollisions::Approximation
FirstOrderTaylorDistributedCollisions::generate_constraint(Opti &ocp, int k) {
  Approximation approx{};
  approx.fot = std::make_shared<Fotdi>();
  approx.fot->f_0 = ocp.parameter(1,1);
  approx.fot->jac_f_0 = ocp.parameter(1,4);

  const auto &xy_r1 = MX::sym("xy_r1", 2,1); //symbolic
  const auto &X0_r1 = *robot->traj0.X0_;
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
