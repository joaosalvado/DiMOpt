#include "integration.hpp"



MX mropt::util::rk4(const Function &f, const MX &dt, const MX &x, const MX &u)
{
  auto k1 = f({{x}, {u}});
  auto k2 = f({{x + ((double)1 / 2) * dt * k1[0]}, {u}});
  auto k3 = f({{x + ((double)1 / 2) * dt * k2[0]}, {u}});
  auto k4 = f({{x + dt * k3[0]}, {u}});
  return  ((double)1 / 6) * dt * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
}

DM mropt::util::rk4_num(const Function &f, const DM &dt, const DM &x, const DM &u)
{

  auto k1 = f(std::vector<casadi::DM>{{x}, {u}});
  auto k2 = f(std::vector<casadi::DM>{{x + ((double)1 / 2) * dt * k1[0]}, {u}});
  auto k3 = f(std::vector<casadi::DM>{{x + ((double)1 / 2) * dt * k2[0]}, {u}});
  auto k4 = f(std::vector<casadi::DM>{{x + dt * k3[0]}, {u}});
  return  ((double)1 / 6) * dt * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
}

