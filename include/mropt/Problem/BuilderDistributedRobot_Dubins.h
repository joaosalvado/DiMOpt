//
// Created by ohmy on 2022-02-23.
//

#ifndef MROPT_BUILDERDISTRIBUTEDROBOT_DUBINS_H
#define MROPT_BUILDERDISTRIBUTEDROBOT_DUBINS_H

#include "mropt/Problem/BuilderDistributedRobot.h"
#include "mropt/mropt.h"
//template class mropt::Problem::BuilderDistributedRobot<double>;

struct ShapeArgs_type{
    double L;
};
namespace mropt::Problem {
    class BuilderDistributedRobot_Dubins : public BuilderDistributedRobot{
    public :
        BuilderDistributedRobot_Dubins() : BuilderDistributedRobot(){}
        virtual ~BuilderDistributedRobot_Dubins() = default;
        std::shared_ptr<mropt::RobotShape::Footprint> build_robot_shape(ShapeArgs_type* args) override;
        std::shared_ptr<mropt::Dynamics::ode> build_ode(
                const std::shared_ptr<mropt::StateSpace::State>& ss,
                const std::shared_ptr<mropt::ControlSpace::Control>& cs,
                const std::shared_ptr<mropt::RobotShape::Footprint>& shape
        ) override;
    };

}


#endif //MROPT_BUILDERDISTRIBUTEDROBOT_DUBINS_H
