#ifndef ODE_H
#define ODE_H
#pragma once

#include "mropt/ControlSpace/Control.hpp"
#include "mropt/StateSpace/State.hpp"
#include "mropt/RobotShape/Footprint.h"
#include <casadi/casadi.hpp>
using namespace casadi;

namespace mropt::Dynamics {
class ode {
protected:
  friend class Transcription;
  friend class OdeApprox;
  casadi::Slice all;
  casadi::Function f_;
  casadi::Function jac_f_;
  ode() = default;
  ode(const std::shared_ptr<mropt::ControlSpace::Control> &cs,
      const std::shared_ptr<mropt::StateSpace::State> &ss,
      const std::shared_ptr<mropt::RobotShape::Footprint> &footprint)
      : control_space_(cs), state_space_(ss) {}

public:
  std::shared_ptr<mropt::ControlSpace::Control> control_space_;
  std::shared_ptr<mropt::StateSpace::State> state_space_;
  std::shared_ptr<mropt::RobotShape::Footprint> footprint_;
  casadi::Function f() const { return f_; };
  casadi::MX f(const casadi::MX &x_ode, const casadi::MX &u_ode) {
    const auto &result = f_({{x_ode}, {u_ode}});
    return result[0];
  }
  casadi::DM f(const casadi::DM &x_r, const casadi::DM &u_r) {
    const auto &result = f_(std::vector<casadi::DM>{{x_r}, {u_r}});
    return result[0];
  }

  casadi::DM jac_f(const casadi::DM &x_r, const casadi::DM &u_r) {
    const auto &result = jac_f_(std::vector<casadi::DM>{{x_r}, {u_r}, {}});
    return result[0];
  }

  virtual ~ode() = default;
  virtual std::shared_ptr<mropt::Dynamics::ode> clone(
      const std::shared_ptr<mropt::StateSpace::State> &state,
      const std::shared_ptr<mropt::ControlSpace::Control> &control,
      const std::shared_ptr<mropt::RobotShape::Footprint> &footprint) const = 0;
};
}



#endif