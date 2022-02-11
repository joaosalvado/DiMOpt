#include "OdeApprox.hpp"

using namespace mropt::Dynamics;

OdeApprox::~OdeApprox()
{
  for(auto *f_ : this->fv_){
    delete f_;
  }
}

std::list<MX> OdeApprox::get_trust_region_constraints_bounded(){
    auto constraints = get_trust_region_constraints();
    for(auto &constraint : constraints){
        constraint = constraint - trust_region_.phi_;
    }
    return constraints;
}

void OdeApprox::set_sym_trajectory0(const std::shared_ptr<MX> &X0, const std::shared_ptr<MX> &U0) {
  X0_ = X0;
  U0_ = U0;
  if (ode_->state_space_->Nx() != X0_->size2())
  {
    std::cerr << "[OdeApprox] X0 has not the right size!" << std::endl;
  }
  if (ode_->control_space_->Nu() != U0_->size2())
  {
    std::cerr << "[OdeApprox] U0 has not the right size!" << std::endl;
  }
  if(X0_->size1() == U0_->size1() == 1){
    std::cerr << " [OdeApprox] trajectory <X0, U0> must be a vector, wrong dims" << std::endl;
  }
}

