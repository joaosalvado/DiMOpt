//
// Created by ohmy on 2021-09-11.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLORDECOUPLEDCOLLISIONS_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLORDECOUPLEDCOLLISIONS_H

#include "../CollisionsAugLagrangian.h"
struct Fotd{
  MX f_0;
  MX jac_f_0;
};

namespace mropt::collisions {
class FirstOrderTaylorDecoupledCollisions : public CollisionsAugLagrangian {
public:
  explicit FirstOrderTaylorDecoupledCollisions(const Collisions &Col);
  ~FirstOrderTaylorDecoupledCollisions() = default;
  void convexify(int r) override;
  Approximation generate_constraint(Opti &ocp, int r1, int k) override;
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLORDECOUPLEDCOLLISIONS_H
