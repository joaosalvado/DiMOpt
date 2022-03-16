//
// Created by ohmy on 2022-03-16.
//

#ifndef MROPT_DIFFDRIVEQUAT_H
#define MROPT_DIFFDRIVEQUAT_H

#include "../ode.hpp"
#include "mropt/StateSpace/quaternions/Quat2D.h"
#include "mropt/ControlSpace/R2/VW.hpp"
#include "mropt/RobotShape/CircleRobot.h"

namespace mropt::Dynamics::CarLike {
    class DiffDriveQuat : public ode {
    private:
        double L_;
    protected:
        DiffDriveQuat() : ode() {}
    public:
        DiffDriveQuat(const std::shared_ptr<mropt::ControlSpace::VW> &vw,
                  const std::shared_ptr<mropt::StateSpace::Quat2D> &quat,
                  const std::shared_ptr<mropt::RobotShape::CircleRobot> &cr)
                : ode(vw, quat, cr), L_(cr->getL()) {
            //Model
            SX X_dot = SX::vertcat(
                    {vw->v_ode * quat->qz_ode,
                     vw->v_ode * quat->qw_ode,
                     (1 / L_)* (-quat->qw_ode) * vw->w_ode,
                     (1 / L_)* quat->qz_ode * vw->w_ode});
            f_ = Function("f", {quat->X_ode(), vw->U_ode()}, {X_dot});
            jac_f_ = f_.jacobian();

        }
        std::shared_ptr<mropt::Dynamics::ode> clone(
                const std::shared_ptr<mropt::StateSpace::State> &state,
                const std::shared_ptr<mropt::ControlSpace::Control> &control,
                const std::shared_ptr<mropt::RobotShape::Footprint> &footprint) const override;
        ~DiffDriveQuat() = default;
    };
}

#endif //MROPT_DIFFDRIVEQUAT_H
