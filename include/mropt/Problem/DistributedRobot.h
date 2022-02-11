//
// Created by ohmy on 2021-09-16.
//

#ifndef MROPT_INCLUDE_MROPT_PROBLEM_DISTRIBUTEDROBOT_H
#define MROPT_INCLUDE_MROPT_PROBLEM_DISTRIBUTEDROBOT_H

#include "Robot.hpp"
#include "mropt/Collisions/DistributedCollisions.h"
#include "mropt/Problem/SharedData.h"

namespace mropt::collisions{ class DistributedCollisions;}

namespace mropt::Problem {
    class DistributedRobot : public Robot {
    public:
        friend class SharedData;
        int robot_id;
        std::shared_ptr<mropt::collisions::DistributedCollisions> collisions_d;
        std::shared_ptr<SharedData> data_shared;
        DistributedRobot(int robot_id,
                         const Params &P,
                         const std::shared_ptr<mropt::ControlSpace::Control> &Cs,
                         const std::shared_ptr<mropt::StateSpace::State> &Ss,
                         const std::shared_ptr<mropt::cost::Cost> &Cost,
                         const std::shared_ptr<mropt::freespace::FreeSpace> &Fspace,
                         const std::shared_ptr<mropt::Dynamics::Transcription> &Dynamics,
                         const std::shared_ptr<mropt::RobotShape::Footprint> &Shape,
                         const std::shared_ptr<mropt::collisions::DistributedCollisions> &Collisions);
        virtual ~DistributedRobot() = default;

        void scp_d();
        bool handle_solution_d(double pho);
        double compute_robot_costs_violations_d();
        bool backtrack_d();
        //No collision
        bool prev_solved_dummy_ocp = false;
        void scp_d_nocol();
        bool handle_solution_d_nocol(double pho);
        double compute_robot_costs_violations_d_nocol();
        bool backtrack_d_nocol();

        double *get_x_curr();
        double *get_y_curr();
        int backtrack_counter;
        DM x2param_list;
        std::shared_ptr<mropt::Dynamics::ode> get_ode() { return dynamics->ode_; }

        bool iter1;
    };
}
#endif //MROPT_INCLUDE_MROPT_PROBLEM_DISTRIBUTEDROBOT_H
