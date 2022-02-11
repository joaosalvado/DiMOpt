//
// Created by ohmy on 2021-07-04.
//

#include "Collisions.h"

using namespace mropt::collisions;

casadi::MX Collisions::f(const casadi::MX &x_ode1, const casadi::MX &x_ode2) const
{
  const auto &result = f_({{x_ode1}, {x_ode2}});
  return result[0];
}
casadi::DM Collisions::f(const casadi::DM &x1, const  casadi::DM &x2) const
{
  const auto &result = f_(std::vector<casadi::DM>{{x1}, {x2}});
  return result[0];
}
casadi::DM Collisions::jac_f(const casadi::DM &x1, const  casadi::DM &x2) const
{
  const auto &result = jac_f_(std::vector<casadi::DM>{{x1}, {x2}, {}});
  //const auto &result = jac_f_(std::vector<casadi::DM>{{x1}, {x2}});
  return result[0];
}
casadi::MX Collisions::jac_f(const casadi::MX &x_ode1, const casadi::MX &x_ode2) const
{
  const auto &result = jac_f_({{x_ode1}, {x_ode2}, {}});
  return result[0];
}