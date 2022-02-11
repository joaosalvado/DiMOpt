#include "SE2.hpp"  

using namespace mropt::StateSpace;
	
SE2::~SE2()
{
	

}
std::shared_ptr<State> SE2::clone() const {
  return std::make_shared<SE2>(*this);
}
