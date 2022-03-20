//
// Created by ohmy on 2022-03-20.
//

#include "BuilderRobot_LGL_Dubins.h"

using namespace mropt::Problem;

std::shared_ptr<mropt::Dynamics::Transcription>
BuilderRobot_LGL_Dubins::build_transcription(
        const std::shared_ptr<mropt::Dynamics::OdeApprox> ode_approx,
        const std::shared_ptr<mropt::cost::Cost> cost){
    auto fot = std::dynamic_pointer_cast<mropt::Dynamics::Approx::FirstOrderTaylor>(ode_approx);
    return std::make_shared<mropt::Dynamics::LGLms>(fot, cost, this->n);
}