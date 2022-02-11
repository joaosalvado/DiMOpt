#ifndef FIRSTORDERTAYLOR_H
#define FIRSTORDERTAYLOR_H
#pragma once

#include "../OdeApprox.hpp"

namespace mropt::Dynamics::Approx {
class FirstOrderTaylor : public OdeApprox {
private:
  MX f_x0u0_;
  std::vector<MX> jac_f_x0u0_;
  DM f_x0u0_val;
  std::vector<DM> jac_f_x0u0_val;
public:
  FirstOrderTaylor(const std::shared_ptr<ode> &ode)
      : OdeApprox(ode) {}

  void generate_approx_params(casadi::Opti &ocp) override;
  void generate_approximation() override;
  /**
   * @brief Compute the f_x0u0, jac_f_x0u0, set them on ocp parameters
   *
   * @param ocp
   * @param x0
   * @param u0
   */
  void convexify(casadi::Opti &ocp, DM &x0, DM &u0) override;
  std::list<MX> get_trust_region_constraints() const override;
  virtual ~FirstOrderTaylor() = default;
  std::shared_ptr<OdeApprox> clone(const std::shared_ptr<ode> &ode) const override;
};
}
#endif