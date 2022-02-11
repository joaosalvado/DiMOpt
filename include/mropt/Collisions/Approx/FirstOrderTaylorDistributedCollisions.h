//
// Created by ohmy on 2021-09-19.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLORDISTRIBUTEDCOLLISIONS_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLORDISTRIBUTEDCOLLISIONS_H

#include "mropt/Collisions/DistributedCollisions.h"


struct Fotdi{
  MX f_0;
  MX jac_f_0;
};

namespace mropt::collisions {
class FirstOrderTaylorDistributedCollisions : public DistributedCollisions {
public:
  explicit FirstOrderTaylorDistributedCollisions(const Collisions &Col);
  ~FirstOrderTaylorDistributedCollisions() = default;
  void convexify() override;
  Approximation generate_constraint(Opti &ocp, int k) override;
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLORDISTRIBUTEDCOLLISIONS_H
