#ifndef MULTIPLESHOOTING_H
#define MULTIPLESHOOTING_H
#pragma once

#include <casadi/casadi.hpp>
#include "../Transcription.hpp"

namespace mropt::Dynamics {
class MultipleShooting : public Transcription {
private:
public:
  MultipleShooting(const std::shared_ptr<OdeApprox> &ode_approx)
      : Transcription(ode_approx) {}
  ~MultipleShooting();

  std::vector<MX> get_constraints() override {
    std::vector<MX> constraints{};
    for (int k = 0; k < N; ++k) {
      const auto &x_next = ode_->state_space_->X()(all, k) + integrator(
          ode_->f(),
          (1 / (double) N) * tf,
          ode_->state_space_->X()(all, k),
          ode_->control_space_->U()(all, k));
      constraints.push_back(ode_->state_space_->X()(all, k + 1) - x_next);
    }
    return constraints;
  }

  void set_J_real() override {
    MX g_sum{0.0};
    for (int k = 0; k < N; ++k) {
      const auto &x_next = ode_->state_space_->X()(all, k) + integrator(
          ode_->f(),
          (1 / (double) N) * tf,
          ode_->state_space_->X()(all, k),
          ode_->control_space_->U()(all, k));
      g_sum = g_sum + sum1(ode_->state_space_->X()(all, k + 1) - x_next);
    }
    J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
  }
private:
  std::shared_ptr<Transcription> clone(const std::shared_ptr<OdeApprox> ode_approx) const override;
};
}
#endif