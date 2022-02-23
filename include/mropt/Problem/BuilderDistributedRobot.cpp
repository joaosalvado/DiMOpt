//
// Created by ohmy on 2022-02-23.
//

#include "BuilderDistributedRobot.h"


using namespace mropt::Problem;


std::shared_ptr<mropt::StateSpace::State>
BuilderDistributedRobot::build_state_space() {
    return std::make_shared<mropt::StateSpace::SE2>();
};

std::shared_ptr<mropt::ControlSpace::Control>
BuilderDistributedRobot::build_control_space() {
    return std::make_shared<mropt::ControlSpace::VW>();
}

std::shared_ptr<mropt::Dynamics::OdeApprox>
BuilderDistributedRobot::build_ode_approx(
        const std::shared_ptr<mropt::Dynamics::ode> ode){
    return std::make_shared<mropt::Dynamics::Approx::FirstOrderTaylor>(ode);
}

std::shared_ptr<mropt::Dynamics::Transcription>
BuilderDistributedRobot::build_transcription(
        const std::shared_ptr<mropt::Dynamics::OdeApprox> ode_approx){
    auto fot = std::dynamic_pointer_cast<mropt::Dynamics::Approx::FirstOrderTaylor>(ode_approx);
    return std::make_shared<mropt::Dynamics::MultipleShootingApprox>(fot);
}


mropt::collisions::Collisions
BuilderDistributedRobot::build_collision_type(){
    return mropt::collisions::L2NormCollision();
}
