//
// Created by ohmy on 2021-07-04.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_L2NORM_L2NORMCOLLISION_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_L2NORM_L2NORMCOLLISION_H

#include "mropt/Collisions/Collisions.h"

namespace mropt::collisions {
class L2NormCollision : public Collisions {

public:
  L2NormCollision() : Collisions() {
    const auto &x_ode_1 = casadi::MX::sym("x1", 2, 1); //xy-coord robot1
    const auto &x_ode_2 = casadi::MX::sym("x2", 2, 1); //xy-coord robot2
    const MX &l2norm = -mtimes(transpose(x_ode_1 - x_ode_2), x_ode_1 - x_ode_2);
    f_ = Function("f", {x_ode_1, x_ode_2}, {l2norm});
    jac_f_ = f_.jacobian();
//    const SX &pseudo_jac = transpose(x_ode_1 - x_ode_2) / l2norm;
//    jac_f_ = Function("jac_col", {x_ode_1, x_ode_2}, {pseudo_jac});
  }
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_L2NORM_L2NORMCOLLISION_H
