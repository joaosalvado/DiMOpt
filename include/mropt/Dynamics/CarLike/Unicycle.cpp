//
// Created by ohmy on 2022-03-14.
//

#include "Unicycle.h"

using namespace mropt::Dynamics::CarLike;

std::shared_ptr<mropt::Dynamics::ode> Unicycle::clone(
        const std::shared_ptr<mropt::StateSpace::State> &state,
        const std::shared_ptr<mropt::ControlSpace::Control> &control,
        const std::shared_ptr<mropt::RobotShape::Footprint> &footprint) const {
    return std::make_shared<Unicycle>(
            std::shared_ptr<mropt::ControlSpace::AP>{
                    dynamic_cast<mropt::ControlSpace::AP*>(control.get())},
            std::shared_ptr<mropt::StateSpace::SE2CU>{
                    dynamic_cast<mropt::StateSpace::SE2CU*>(state.get())},
            std::shared_ptr<mropt::RobotShape::CircleRobot>{
                    dynamic_cast<mropt::RobotShape::CircleRobot*>(footprint_.get())});
}
