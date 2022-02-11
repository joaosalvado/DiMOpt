#ifndef MROPT_UTIL_PLOTTER_H
#define MROPT_UTIL_PLOTTER_H
#pragma once

#include <vector>
#include <functional>
#include "integration.hpp"
#include "mropt/Dynamics/ode.hpp"

// class forward
namespace mropt::Problem{
 class CoupledProblem;
 class DecoupledProblem;
}

namespace mropt::util {
class Plotter {
protected:
  std::vector<std::function<void(double, double, double)>> plot_robot;
  std::function<DM(const Function &, const DM &, const DM &, const DM &)> integrator;
  std::vector<std::shared_ptr<mropt::Dynamics::ode>> odes;
  int R{0};
  Slice all;

public:
  friend class mropt::Problem::CoupledProblem;
  friend class mropt::Problem::DecoupledProblem;
  explicit Plotter(int R) : integrator(rk4_num), R(R) {}
  virtual ~Plotter() = default;
  void set_models(const std::vector<std::shared_ptr<mropt::Dynamics::ode>> &odes_) {
    odes = odes_;
    R = odes.size();
  }

  virtual void plot_trajectory(
      std::vector<std::vector<double>> x,
      std::vector<std::vector<double>> y,
      std::vector<std::vector<double>> o,
      std::vector<std::vector<std::vector<double>>> u,
      double time) = 0;

  void plot_swath(
          std::vector<std::vector<double>> x,
            std::vector<std::vector<double>> y,
            std::vector<std::vector<double>> o);

};
}
#endif