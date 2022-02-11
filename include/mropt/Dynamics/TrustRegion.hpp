#ifndef TRUSTREGION_H
#define TRUSTREGION_H
#pragma once

#include "ode.hpp"
using namespace casadi;

namespace mropt::Dynamics {
class TrustRegion {
private:
  friend class OdeApprox;
  double expand_factor{3.0};
  double shrink_factor{0.5};
  double phi_val_{1};
  double phi_val_minimum{1.0e-3};
  Slice all;
  MX phi_;
public:
  ~TrustRegion();

  void setParameters(double tau_plus, double tau_minus) {
    expand_factor = tau_plus;
    shrink_factor = tau_minus;
  }

  void setup(casadi::Opti &ocp) {
    phi_ = ocp.parameter(1);
    ocp.set_value(phi_, phi_val_);
  }

  void expand(Opti &ocp) {
    phi_val_ = expand_factor * phi_val_;
    ocp.set_value(phi_, phi_val_);
  }

  void shrink(Opti &ocp) {
    phi_val_ = shrink_factor * phi_val_;
    //phi_val_ = std::max(phi_val_, phi_val_minimum); //do not allow very small
    ocp.set_value(phi_, phi_val_);
  }

  void reset(Opti &ocp) {
    phi_val_ = 1;
    ocp.set_value(phi_, 1);
  }

  double get_radius() { return phi_val_; }

  std::list<MX> l2_norm_trust_region_constraints(const ode &ode_,
                                                 const std::shared_ptr<MX> &X0_,
                                                 const std::shared_ptr<MX> &U0_) const;
  std::list<MX> l1_norm_trust_region_constraints(const ode &ode_,
                                                 const std::shared_ptr<MX> &X0_,
                                                 const std::shared_ptr<MX> &U0_) const;
};
}
#endif