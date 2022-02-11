#include "VW.hpp"  

using namespace mropt::ControlSpace;

VW::~VW()
{
	
}
std::shared_ptr<Control> VW::clone() const {
  return std::make_shared<VW>(*this);
}
