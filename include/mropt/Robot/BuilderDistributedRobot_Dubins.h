//
// Created by ohmy on 2022-02-23.
//

#ifndef MROPT_BUILDERDISTRIBUTEDROBOT_DUBINS_H
#define MROPT_BUILDERDISTRIBUTEDROBOT_DUBINS_H

#include "BuilderDistributedRobot.h"
#include "mropt/mropt.h"

namespace mropt::Problem {
    class BuilderDistributedRobot_Dubins : public BuilderDistributedRobot{
    double L;
    public :
        BuilderDistributedRobot_Dubins(double L) : BuilderDistributedRobot(){
        this->L = L;
    }
        virtual ~BuilderDistributedRobot_Dubins() = default;
        std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>  create_shape_builder() override;
        std::shared_ptr<mropt::Dynamics::ode> build_ode(
                const std::shared_ptr<mropt::StateSpace::State>& ss,
                const std::shared_ptr<mropt::ControlSpace::Control>& cs,
                const std::shared_ptr<mropt::RobotShape::Footprint>& shape
        ) override;
    };

}


#endif //MROPT_BUILDERDISTRIBUTEDROBOT_DUBINS_H
