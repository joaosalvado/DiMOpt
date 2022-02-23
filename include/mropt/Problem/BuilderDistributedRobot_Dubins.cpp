//
// Created by ohmy on 2022-02-23.
//

#include "BuilderDistributedRobot_Dubins.h"
#include "mropt/mropt.h"
using namespace mropt::Problem;

std::shared_ptr<mropt::RobotShape::Footprint>
  BuilderDistributedRobot_Dubins::build_robot_shape(ShapeArgs_type* arg){
    return std::make_shared<mropt::RobotShape::CircleRobot>(arg->L/2);
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