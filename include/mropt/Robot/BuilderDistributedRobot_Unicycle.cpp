//
// Created by ohmy on 2022-03-14.
//

#include "BuilderDistributedRobot_Unicycle.h"

#include "mropt/StateSpace/SE2CU/SE2CU.h"
#include "mropt/ControlSpace/R2/AP.h"
#include "mropt/Dynamics/CarLike/Unicycle.h"



using namespace mropt::Problem;

std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>
BuilderDistributedRobot_Unicycle::create_shape_builder(){
    return [this](){return std::make_shared<mropt::RobotShape::CircleRobot>(L/2); };
}

std::shared_ptr<mropt::Dynamics::ode>
BuilderDistributedRobot_Unicycle::build_ode(
        const std::shared_ptr<mropt::StateSpace::State>& ss,
        const std::shared_ptr<mropt::ControlSpace::Control>& cs,
        const std::shared_ptr<mropt::RobotShape::Footprint>& shape
){
    auto se2cu = std::dynamic_pointer_cast<mropt::StateSpace::SE2CU>(ss);
    auto ap = std::dynamic_pointer_cast<mropt::ControlSpace::AP>(cs);
    auto cr = std::dynamic_pointer_cast<mropt::RobotShape::CircleRobot>(shape);
    return std::make_shared<mropt::Dynamics::CarLike::Unicycle>(ap, se2cu, cr);
}
