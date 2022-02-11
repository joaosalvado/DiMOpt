/**
 * @file Cost.hpp
 * @author João Salvado (joao.salvado@protonmail.com)
 * @brief This definies a standard QP cost arround control and state space
 * weights and standard values should be defined in statespace and control space.
 * For different costs create a derived class and override generate_cost() function
 * @version 0.1
 * @date 2021-06-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef COST_H
#define COST_H
#pragma once

#include "mropt/ControlSpace/Control.hpp"
#include "mropt/StateSpace/State.hpp"
#include <casadi/casadi.hpp>
#include "../util/integration.hpp"
using namespace casadi;
// class forward
namespace mropt::Problem{
  class Problem;
  class Robot;
}

namespace mropt::cost {
class Cost {
protected:
  friend class mropt::Problem::Robot;
  Slice all;
  Function l_, J_;
  mropt::StateSpace::State &state_space_;
  mropt::ControlSpace::Control &control_space_;

  std::function<MX(const Function &, const MX &, const MX &, const MX &)> integrator;
  MX integrated_cost(MX t0, MX tf, int N) {
    //Integrated cost
    MX _J = 0;
    for (int k = 0; k < N; ++k) {
      _J = _J + integrator(l_, (tf - t0) / (double) N, state_space_.X()(all, k), control_space_.U()(all, k));
    }
    MX params = MX::vertcat({t0, tf});
    J_ = Function("J", {state_space_.X(), control_space_.U(), params}, {_J});
    return _J;
  }
public:
  Cost(
      mropt::ControlSpace::Control &cs,
      mropt::StateSpace::State &ss)
      : control_space_(cs), state_space_(ss), integrator(mropt::util::rk4) {
    //integrator = rk4;
    set_cost();
  }
  Cost(const Cost &) = delete;
  Cost &operator=(const Cost &) = delete;

  casadi::Function l() const { return l_; };
  casadi::MX l(const casadi::MX &x_ode, const casadi::MX &u_ode) {
    const auto &result = l_({{x_ode}, {u_ode}});
    return result[0];
  }
  casadi::DM l(const casadi::DM &x_r, const casadi::DM &u_r) {
    const auto &result = l_(std::vector<casadi::DM>{{x_r}, {u_r}});
    return result[0];
  }
  casadi::DM J(const casadi::DM &x_r, const casadi::DM &u_r, const casadi::DM &p) {
    const auto &result = J_(std::vector<casadi::DM>{{x_r}, {u_r}, {p}});
    return result[0];
  }

  void set_cost(Function l) {
    this->l_ = l;
  }

  void set_cost() {
    l_ = Function("l", {state_space_.X_ode(), control_space_.U_ode()}, {generate_cost()});
  }

  /**
   * @brief Note that one is required to set_cost() to update cost function
   *
   * @return SX - cost symbolic equation
   */
  SX generate_cost();

  Cost &set_integrator(std::function<MX(const Function &, const MX &, const MX &, const MX &)> integr) {
    integrator = integr;
    return *this;
  }

  ~Cost() {}
};
}
#endif