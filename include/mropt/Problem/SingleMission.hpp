#ifndef SINGLEMISSION_H
#define SINGLEMISSION_H
#pragma once

#include "mropt/StateSpace/State.hpp"
#include "mropt/FreeSpace/FreeSpace.hpp"
#include <list>
using namespace casadi;
// Class forwarding
namespace mropt::collisions{
 class CollisionsApprox;
 class CollisionsAugLagrangian;
}

namespace mropt::Problem {
class SingleMission {
protected:
  friend class mropt::collisions::CollisionsApprox;
  friend class mropt::collisions::CollisionsAugLagrangian;
  friend class Robot;
  std::vector<double> x0, xf;
  std::vector<mropt::freespace::FreeSpace::PolygonAssignment> pas;
  std::vector<std::vector<int>> pol_alloc;
  mropt::StateSpace::State &ss;
public:
  explicit SingleMission(mropt::StateSpace::State &ss) : ss(ss) {};

  void init(std::vector<double> x);
  void goal(std::vector<double> x);
  void path(const std::vector<mropt::freespace::FreeSpace::PolygonAssignment> &pas_) {
    pas = pas_;
    compute_polygon_assignments();
  }

  virtual std::list<MX> get_constraints();
  MX get_cost();

  void set_problem(MX &J_model, Opti &ocp);

  void compute_polygon_assignments();

private:
  void addNoise(std::vector<double> &x);

};
}
#endif