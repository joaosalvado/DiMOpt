//
// Created by ohmy on 2022-03-16.
//

#include "DiffDriveQuat.h"

using namespace mropt::Dynamics::CarLike;

std::shared_ptr<mropt::Dynamics::ode> DiffDriveQuat::clone(
        const std::shared_ptr<mropt::StateSpace::State> &state,
        const std::shared_ptr<mropt::ControlSpace::Control> &control,
        const std::shared_ptr<mropt::RobotShape::Footprint> &footprint) const {
    return std::make_shared<DiffDriveQuat>(
            std::shared_ptr<mropt::ControlSpace::VW>{
                    dynamic_cast<mropt::ControlSpace::VW*>(control.get())},
            std::shared_ptr<mropt::StateSpace::Quat2D>{
                    dynamic_cast<mropt::StateSpace::Quat2D*>(state.get())},
            std::shared_ptr<mropt::RobotShape::CircleRobot>{
                    dynamic_cast<mropt::RobotShape::CircleRobot*>(footprint_.get())});
}
