#include "VW.hpp"  

using namespace mropt::ControlSpace;

VW::~VW()
{
	
}
std::shared_ptr<Control> VW::clone() const {
  return std::make_shared<VW>(*this);
}

void VW::initial_guess(
        const std::vector<double> &x0,
        const std::vector<double> &xf,
        casadi::DM &U_guess){

    auto v_fake = transpose(DM({std::vector(Nu(), this->u_v_std)}));
    auto u_fake = transpose(DM({std::vector(Nu(), this->u_w_std)}));

    U_guess = casadi::DM({v_fake.get_elements(), u_fake.get_elements()});

}
