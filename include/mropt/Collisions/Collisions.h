//
// Created by ohmy on 2021-07-04.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_COLLISIONS_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_COLLISIONS_H

#include <casadi/casadi.hpp>
#include "mropt/StateSpace/State.hpp"
using namespace casadi;

namespace mropt::collisions {
class Collisions {
protected:
  casadi::Function f_;
  casadi::Function jac_f_;

public:
  Function f() const { return f_; };
  MX f(const MX &x_ode1, const MX &x_ode2) const;
  DM f(const DM &x1, const DM &x2) const;
  DM jac_f(const DM &x1, const DM &x2) const;
  MX jac_f(const MX &x1, const MX &x2) const;

  double threshold{0.03};
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_COLLISIONS_H
