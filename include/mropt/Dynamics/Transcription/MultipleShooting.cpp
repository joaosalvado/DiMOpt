#include "MultipleShooting.hpp"


using namespace mropt::Dynamics;
MultipleShooting::~MultipleShooting()
{

}
std::shared_ptr<mropt::Dynamics::Transcription> MultipleShooting::clone(
    const std::shared_ptr<mropt::Dynamics::OdeApprox> ode_approx) const {
  return std::make_shared<MultipleShooting>(ode_approx);
}
