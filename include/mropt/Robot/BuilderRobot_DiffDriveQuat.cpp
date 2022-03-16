//
// Created by ohmy on 2022-03-16.
//

#include "BuilderRobot_DiffDriveQuat.h"
#include "mropt/StateSpace/quaternions/Quat2D.h"
#include "mropt/Dynamics/CarLike/DiffDriveQuat.h"


using namespace mropt::Problem;

std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>
BuilderRobot_DiffDriveQuat::create_shape_builder(){
    return [this]()->std::shared_ptr<mropt::RobotShape::CircleRobot>
    {return std::make_shared<mropt::RobotShape::CircleRobot>(L/2); };
}

std::shared_ptr<mropt::Dynamics::ode>
BuilderRobot_DiffDriveQuat::build_ode(
        const std::shared_ptr<mropt::StateSpace::State>& ss,
        const std::shared_ptr<mropt::ControlSpace::Control>& cs,
        const std::shared_ptr<mropt::RobotShape::Footprint>& shape
){
    auto quat = std::dynamic_pointer_cast<mropt::StateSpace::Quat2D>(ss);
    auto vw = std::dynamic_pointer_cast<mropt::ControlSpace::VW>(cs);
    auto cr = std::dynamic_pointer_cast<mropt::RobotShape::CircleRobot>(shape);
    return std::make_shared<mropt::Dynamics::CarLike::DiffDriveQuat>(vw, quat, cr);
}

std::shared_ptr<mropt::StateSpace::State>
BuilderRobot_DiffDriveQuat::build_state_space(){
    return std::make_shared<mropt::StateSpace::Quat2D>();
}