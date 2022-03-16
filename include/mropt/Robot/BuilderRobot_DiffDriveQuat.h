//
// Created by ohmy on 2022-03-16.
//

#ifndef MROPT_BUILDERROBOT_DIFFDRIVEQUAT_H
#define MROPT_BUILDERROBOT_DIFFDRIVEQUAT_H



#include "BuilderDistributedRobot.h"
#include "mropt/mropt.h"

namespace mropt::Problem {
    class BuilderRobot_DiffDriveQuat : public BuilderDistributedRobot{
        double L;
    public :
        BuilderRobot_DiffDriveQuat(double L) : BuilderDistributedRobot(){
            this->L = L;
        }
        virtual ~BuilderRobot_DiffDriveQuat() = default;
        std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>  create_shape_builder() override;
        std::shared_ptr<mropt::Dynamics::ode> build_ode(
                const std::shared_ptr<mropt::StateSpace::State>& ss,
                const std::shared_ptr<mropt::ControlSpace::Control>& cs,
                const std::shared_ptr<mropt::RobotShape::Footprint>& shape
        ) override;
        std::shared_ptr<mropt::StateSpace::State> build_state_space() override;
    };

}



#endif //MROPT_BUILDERROBOT_DIFFDRIVEQUAT_H
