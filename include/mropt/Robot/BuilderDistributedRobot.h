//
// Created by ohmy on 2022-02-23.
//

#ifndef MROPT_BUILDERDISTRIBUTEDROBOT_H
#define MROPT_BUILDERDISTRIBUTEDROBOT_H

#include "mropt/Problem/DistributedRobot.h"
#include "mropt/mropt.h"


namespace mropt::Problem {
    class BuilderDistributedRobot{
        std::shared_ptr<DistributedRobot> robot_d;
        mropt::collisions::Collisions col_type;
    public:
        BuilderDistributedRobot(){
            robot_d = std::make_unique<DistributedRobot>();
        }
        virtual ~BuilderDistributedRobot() = default;
    public:
        virtual std::shared_ptr<mropt::StateSpace::State> build_state_space();
        virtual std::shared_ptr<mropt::ControlSpace::Control> build_control_space();
        virtual std::function<std::shared_ptr<mropt::RobotShape::Footprint>()>  create_shape_builder() = 0;
        virtual std::shared_ptr<mropt::Dynamics::ode> build_ode(
                const std::shared_ptr<mropt::StateSpace::State>& ss,
                const std::shared_ptr<mropt::ControlSpace::Control>& cs,
                const std::shared_ptr<mropt::RobotShape::Footprint>& shape) = 0;
        virtual std::shared_ptr<mropt::Dynamics::OdeApprox> build_ode_approx(
                const std::shared_ptr<mropt::Dynamics::ode> ode );
        virtual std::shared_ptr<mropt::Dynamics::Transcription> build_transcription(
                const std::shared_ptr<mropt::Dynamics::OdeApprox> ode_approx);
        virtual mropt::collisions::Collisions build_collision_type();

        std::shared_ptr<DistributedRobot> getDistributedRobot(){
            return std::move(robot_d);
        }

        void make_robot(
                int robot_id,
                mropt::Problem::Robot::Params params){
            robot_d->p_ = params;
            robot_d->robot_id = robot_id;
            // State Space
            robot_d->ss = build_state_space();
            // Control Space
            robot_d->cs = build_control_space();
            // Robot Shape
            auto build_robot_shape = create_shape_builder();
            robot_d->shape = build_robot_shape();
            // Dynamics
            auto model = build_ode(robot_d->ss, robot_d->cs, robot_d->shape);
            auto approx_model = build_ode_approx(model);
            auto transcription = build_transcription(approx_model);
            robot_d->dynamics = transcription;
            // Collisions
            col_type = build_collision_type();
            robot_d->collisions_d = std::make_shared<mropt::collisions::FirstOrderTaylorDistributedCollisions>(col_type);
            // Free Space
            robot_d->fspace = std::make_shared<mropt::freespace::FreeSpace>(*robot_d->ss);
            // Cost
            robot_d->cost = std::make_shared<mropt::cost::Cost>(*robot_d->cs, *robot_d->ss);
            robot_d->setup();
        }

    };
}


#endif //MROPT_BUILDERDISTRIBUTEDROBOT_H
