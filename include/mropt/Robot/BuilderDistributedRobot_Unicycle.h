//
// Created by ohmy on 2022-03-14.
//

#ifndef MROPT_BUILDERDISTRIBUTEDROBOT_UNICYCLE_H
#define MROPT_BUILDERDISTRIBUTEDROBOT_UNICYCLE_H

#include "BuilderDistributedRobot.h"
#include "mropt/mropt.h"

namespace mropt::Problem {
    class BuilderDistributedRobot_Unicycle : public BuilderDistributedRobot{
        double L;
    public:
        BuilderDistributedRobot_Unicycle(double L) : BuilderDistributedRobot(){
            this->L = L;
        }
        virtual ~BuilderDistributedRobot_Unicycle() = default;
        std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>  create_shape_builder() override;
        std::shared_ptr<mropt::Dynamics::ode> build_ode(
                const std::shared_ptr<mropt::StateSpace::State>& ss,
                const std::shared_ptr<mropt::ControlSpace::Control>& cs,
                const std::shared_ptr<mropt::RobotShape::Footprint>& shape
        ) override;

        std::shared_ptr<mropt::StateSpace::State> build_state_space() override;
        std::shared_ptr<mropt::ControlSpace::Control> build_control_space() override;
    };

}


#endif //MROPT_BUILDERDISTRIBUTEDROBOT_UNICYLCE_H
