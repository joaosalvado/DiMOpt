//
// Created by ohmy on 2022-03-14.
//

#ifndef MROPT_UNICYCLE_H
#define MROPT_UNICYCLE_H
#pragma once

#include "../ode.hpp"
#include "mropt/StateSpace/SE2CU/SE2CU.h"
#include "mropt/ControlSpace/R2/AP.h"
#include "mropt/RobotShape/CircleRobot.h"

namespace mropt::Dynamics::CarLike {
    class Unicycle : public ode{
        double L_;
    public:
        Unicycle(const std::shared_ptr<mropt::ControlSpace::AP> &ap,
                  const std::shared_ptr<mropt::StateSpace::SE2CU> &se2cu,
                  const std::shared_ptr<mropt::RobotShape::CircleRobot> &cr)
                : ode(ap, se2cu, cr), L_(cr->getL()) {
            //Model
            SX X_dot = SX::vertcat(
                    {se2cu->v_ode * cos(se2cu->o_ode),
                     se2cu->v_ode * sin(se2cu->o_ode),
                     (1 / L_) * se2cu->v_ode * se2cu->p_ode,
                     ap->a_ode,
                     ap->p_ode
                    });
            f_ = Function("f", {se2cu->X_ode(), ap->U_ode()}, {X_dot});
            jac_f_ = f_.jacobian();

        }

        ~Unicycle() = default;

        std::shared_ptr<mropt::Dynamics::ode> clone(
                const std::shared_ptr<mropt::StateSpace::State> &state,
                const std::shared_ptr<mropt::ControlSpace::Control> &control,
                const std::shared_ptr<mropt::RobotShape::Footprint> &footprint) const override;
    protected:
        Unicycle() :ode(){ }
    };
}
#endif //MROPT_UNICYCLE_H
