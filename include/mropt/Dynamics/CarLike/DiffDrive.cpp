#include "DiffDrive.hpp"  
	
using namespace mropt::Dynamics::CarLike;

std::shared_ptr<mropt::Dynamics::ode> DiffDrive::clone(
    const std::shared_ptr<mropt::StateSpace::State> &state,
    const std::shared_ptr<mropt::ControlSpace::Control> &control,
    const std::shared_ptr<mropt::RobotShape::Footprint> &footprint) const {
  return std::make_shared<DiffDrive>(
      std::shared_ptr<mropt::ControlSpace::VW>{
              dynamic_cast<mropt::ControlSpace::VW*>(control.get())},
      std::shared_ptr<mropt::StateSpace::SE2>{
              dynamic_cast<mropt::StateSpace::SE2*>(state.get())},
      std::shared_ptr<mropt::RobotShape::CircleRobot>{
              dynamic_cast<mropt::RobotShape::CircleRobot*>(footprint_.get())});
}
