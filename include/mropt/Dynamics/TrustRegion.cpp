#include "TrustRegion.hpp"  

using namespace mropt::Dynamics;

TrustRegion::~TrustRegion()
{
	
}

std::list<MX> TrustRegion::l2_norm_trust_region_constraints(
    const ode &ode_,
    const std::shared_ptr<MX> &X0_,
    const std::shared_ptr<MX> &U0_) const
{
    std::list<MX> constraints;
    // State
    const auto &X = ode_.state_space_->X();
    for (int x_id = 0; x_id < ode_.state_space_->nx(); ++x_id)
    {
        constraints.push_back(
         ( X(x_id, 1) -  (*X0_)(x_id, 1) ) * ( (X(x_id, 1) - (*X0_)(x_id, 1)) ) );
    }
    // Control
    const auto &U = ode_.control_space_->U();
    for (int u_id = 0; u_id < ode_.control_space_->nu(); ++u_id)
    {
        constraints.push_back(
              ( U(u_id, all) - (*U0_)(u_id, all) ) * ( (U(u_id, all) - (*U0_)(u_id, all)) ) );
    }

    return constraints;
}


std::list<MX> TrustRegion::l1_norm_trust_region_constraints(
    const ode &ode_,
    const std::shared_ptr<MX> &X0_,
    const std::shared_ptr<MX> &U0_) const
{
    std::list<MX> constraints;
    // State
    const auto &X = ode_.state_space_->X();
    for (int x_id = 0; x_id < ode_.state_space_->nx(); ++x_id)
    {
        constraints.push_back( X(x_id, 1) - (*X0_)(x_id, 1) );
        constraints.push_back( -X(x_id, 1) + (*X0_)(x_id, 1) );
    }
    // Control
    const auto &U = ode_.control_space_->U();
    for (int u_id = 0; u_id < ode_.control_space_->nu(); ++u_id)
    {
        constraints.push_back(  U(u_id, all) - (*U0_)(u_id, all) );
        constraints.push_back( -U(u_id, all) + (*U0_)(u_id, all) );
    }

    return constraints;
}
