#ifndef TRANSCRIPTION_H
#define TRANSCRIPTION_H
#pragma once

#include <casadi/casadi.hpp>
#include "mropt/util/integration.hpp"
#include "OdeApprox.hpp"
#include "ode.hpp"

using namespace casadi;

// Class Forward
namespace mropt::Problem {class Robot;}
namespace mropt::Problem {class CoupledProblem;}
namespace mropt::Problem {class DecoupledProblem;}
namespace mropt::Problem {class DistributedRobot;}

namespace mropt::Dynamics {
class Transcription {
protected:
  friend class mropt::Problem::CoupledProblem;
  friend class mropt::Problem::DecoupledProblem;
  friend class mropt::Problem::Robot;
  friend class mropt::Problem::DistributedRobot;
  Slice all;
  std::shared_ptr<OdeApprox> ode_approx_;
  Function J_real_;
  Function J_max_;
  Function J_model_;
  std::shared_ptr<ode> ode_;
  int N{0};
  MX t0, tf;
  std::function<MX(const Function &, const MX &, const MX &, const MX &)> integrator;

public:
  explicit Transcription(const std::shared_ptr<OdeApprox> &ode)
      : ode_approx_(ode), ode_(ode->get_ode()), integrator(mropt::util::rk4) {
    //integrator = rk4;
  };

  Transcription &set_integrator(std::function<MX(const Function &, const MX &, const MX &, const MX &)> integr) {
    integrator = integr;
    return *this;
  }
  virtual ~Transcription();

private:
  void setup(
      casadi::Opti &ocp,
      std::shared_ptr<MX> X0,
      std::shared_ptr<MX> U0,
      double t0_, double tf_, int N_) {
    t0 = ocp.parameter(1);
    tf = ocp.parameter(1);
    N = N_;
    ocp.set_value(t0, t0_);
    ocp.set_value(tf, tf_);
    ode_approx_->setup(ocp, X0, U0);
    set_J_real();
  }

  casadi::DM J_real(const casadi::DM &x_r, const casadi::DM &u_r, const casadi::DM &p) {
    const auto &result = J_real_(std::vector<casadi::DM>{{x_r}, {u_r}, {p}});
    return result[0];
  }

  casadi::DM J_max(const casadi::DM &x_r, const casadi::DM &u_r, const casadi::DM &p) {
    const auto &result = J_max_(std::vector<casadi::DM>{{x_r}, {u_r}, {p}});
    return result[0];
  }

  casadi::DM J_model(const casadi::DM &x_r, const casadi::DM &u_r, const casadi::DM &p) {
    const auto &result = J_model_(std::vector<casadi::DM>{{x_r}, {u_r}, {p}});
    return result[0];
  }

  virtual std::vector<MX> get_constraints() = 0;

  std::list<MX> get_trust_region_constraints() {
    return ode_approx_->get_trust_region_constraints_bounded();
  }

  /**
   * @brief compute the approximation arround the trajectory x0 and u0 and set parameters ocp
   * Note: approximation is parametric in ocp, so by computing and setting the params we are
   * approximating
   *
   * @param ocp
   * @param x0
   * @param u0
   */
  void convexify(casadi::Opti &ocp, DM &x0, DM &u0) {
    ode_approx_->convexify(ocp, x0, u0);
  }

  virtual void set_J_real() = 0;

  TrustRegion &trustRegion() { return ode_approx_->trust_region_; }
  virtual std::shared_ptr<Transcription> clone(
      const std::shared_ptr<OdeApprox> ode_approx) const = 0;
};
}
#endif