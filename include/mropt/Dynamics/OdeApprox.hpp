#ifndef ODEAPPROX_H
#define ODEAPPROX_H
#pragma once

#include "ode.hpp"
#include <casadi/casadi.hpp>
#include "TrustRegion.hpp"

using namespace casadi;
//namespace mropt::Problem{ template <class shape_args> class BuilderDistributedRobot;}
namespace mropt::Dynamics {
class OdeApprox {
protected:
  friend class Transcription;
  //template <class shape_args> friend class mropt::Problem::BuilderDistributedRobot;
  Slice all;
  std::shared_ptr<ode> ode_;
  std::shared_ptr<MX> X0_;
  std::shared_ptr<MX> U0_;
  std::vector<Function *> fv_;
  TrustRegion trust_region_;
  //casadi::Opti& ocp;
private:
    OdeApprox(){}
    void setOde(const std::shared_ptr<ode> &ode) {
        ode_ = ode;
    }

public:
  explicit OdeApprox(const std::shared_ptr<ode> &ode) : ode_(ode) {}

  virtual void generate_approx_params(casadi::Opti &ocp) = 0;
  void set_sym_trajectory0(const std::shared_ptr<MX> &X0, const std::shared_ptr<MX> &U0);
  virtual void generate_approximation() = 0;
  virtual void convexify(casadi::Opti &ocp, DM &x0, DM &u0) = 0;

  void setup(Opti &ocp, std::shared_ptr<MX> X0, std::shared_ptr<MX> U0) {
    while (!fv_.empty()) {
      delete fv_.back();
      fv_.pop_back();
    }
    generate_approx_params(ocp);
    set_sym_trajectory0(X0, U0);
    generate_approximation();
    trust_region_.setup(ocp);
  }

  virtual std::list<MX> get_trust_region_constraints() const = 0;
  std::list<MX> get_trust_region_constraints_bounded();

  //Utils
  Function *fapprox(int k) { return fv_[k]; }
  int get_nx() const { return ode_->state_space_->nx(); }
  int get_nu() const { return ode_->control_space_->nu(); }
  int get_N() const { return ode_->state_space_->Nx() - 1; }
  std::shared_ptr<ode> get_ode() const { return ode_; }

  virtual ~OdeApprox();
  virtual std::shared_ptr<OdeApprox> clone(
      const std::shared_ptr<ode> &ode) const = 0;
};
}
#endif