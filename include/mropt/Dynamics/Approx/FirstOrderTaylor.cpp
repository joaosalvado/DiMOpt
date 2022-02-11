#include "FirstOrderTaylor.hpp"

using namespace mropt::Dynamics::Approx;
void FirstOrderTaylor::generate_approx_params(casadi::Opti &ocp)
{
  if(!jac_f_x0u0_.empty()) {jac_f_x0u0_.clear(); jac_f_x0u0_val.clear();}
  auto nx = ode_->state_space_->nx();
  auto Nx = ode_->state_space_->Nx();
  auto nu = ode_->control_space_->nu();
//  auto Nu = ode_.control_space_->Nu();
// Model Approx
  f_x0u0_ = ocp.parameter(nx, Nx-1);
  for (int i = 0; i < Nx-1; ++i)
  {
    jac_f_x0u0_.push_back(ocp.parameter(nx, nx + nu));
  }
  f_x0u0_val = DM(nx, Nx-1);
  jac_f_x0u0_val = std::vector<DM>(Nx-1, DM(nx, nx + nu));
}

void FirstOrderTaylor::generate_approximation()
{
  auto nx = get_nx();
  auto nu = get_nu();
  auto N = get_N();
  auto x_ode_mx = MX::sym("x", nx);
  auto u_ode_mx = MX::sym("u", nu);
  for (int k = 0; k < N; ++k)
  {
    const MX &x_dot_taylor = f_x0u0_(all, k) + mtimes(jac_f_x0u0_[k], MX::vertcat({x_ode_mx - (*X0_)(all, k), u_ode_mx - (*U0_)(all, k)}));
    auto *f_approx = new Function("f_approx", {{x_ode_mx}, {u_ode_mx}}, {x_dot_taylor});
    this->fv_.push_back(f_approx);
  }
}

void FirstOrderTaylor::convexify(casadi::Opti &ocp, DM &x0, DM &u0)
{
  auto N = get_N();
  for (int k = 0; k < N; ++k)
  {
    f_x0u0_val(all, k) = ode_->f( x0(all, k), u0(all, k) );
    jac_f_x0u0_val[k] = ode_->jac_f( DM{x0(all, k)}, DM{u0(all, k)} );
    ocp.set_value(jac_f_x0u0_[k], jac_f_x0u0_val[k]);
  }
  ocp.set_value(f_x0u0_, f_x0u0_val);
}

std::list<MX> FirstOrderTaylor::get_trust_region_constraints() const {
  return this->trust_region_.l1_norm_trust_region_constraints(*ode_, X0_, U0_);
}

std::shared_ptr<mropt::Dynamics::OdeApprox> FirstOrderTaylor::clone(const std::shared_ptr<ode> &ode) const {
  return std::make_shared<FirstOrderTaylor>(ode);
}
