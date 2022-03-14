//
// Created by ohmy on 2022-02-23.
//

#include "BuilderDistributedRobot_Dubins.h"
using namespace mropt::Problem;

std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>
  BuilderDistributedRobot_Dubins::create_shape_builder(){
    return [this]()->std::shared_ptr<mropt::RobotShape::CircleRobot>
            {return std::make_shared<mropt::RobotShape::CircleRobot>(L/2); };
}

std::shared_ptr<mropt::Dynamics::ode>
  BuilderDistributedRobot_Dubins::build_ode(
        const std::shared_ptr<mropt::StateSpace::State>& ss,
        const std::shared_ptr<mropt::ControlSpace::Control>& cs,
        const std::shared_ptr<mropt::RobotShape::Footprint>& shape
){
    auto se2 = std::dynamic_pointer_cast<mropt::StateSpace::SE2>(ss);
    auto vw = std::dynamic_pointer_cast<mropt::ControlSpace::VW>(cs);
    auto cr = std::dynamic_pointer_cast<mropt::RobotShape::CircleRobot>(shape);
    return std::make_shared<mropt::Dynamics::CarLike::DiffDrive>(vw, se2, cr);
}