//
// Created by ohmy on 2021-07-04.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLOR_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLOR_H

#include "mropt/Collisions/CollisionsApprox.h"

struct Fot{
  MX f_0;
  MX jac_f_0;
};

namespace mropt::collisions {
class FirstOrderTaylorCollisions : public CollisionsApprox {
public:
  explicit FirstOrderTaylorCollisions(
      const Collisions &col) : CollisionsApprox(col) {};
  void convexify(Opti &ocp) override;
  void generate_constraint(casadi::Opti &ocp, int r1, int r2, int k) override;
private:
public:
  //virtual ~FirstOrderTaylorCollisions() {}
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_APPROX_FIRSTORDERTAYLOR_H
