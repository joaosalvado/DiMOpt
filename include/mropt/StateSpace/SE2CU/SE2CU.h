//
// Created by ohmy on 2022-03-14.
//

#ifndef MROPT_SE2CU_H
#define MROPT_SE2CU_H
#pragma once

#include "../State.hpp"
using namespace casadi;
namespace mropt::StateSpace {


    class SE2CU : public State{
        double w_x = 0.0;
        double w_y = 0.0;
        double w_o = 0.0;
        double w_v = 0.0;
        double w_p = 0.0;
        double x_std = 0.0;
        double y_std = 0.0;
        double o_std = 0.0;
        double v_std = 0.5;
        double p_std = 0.0;
    public:
        SE2CU() : State() { nx_ = 5; }
        virtual ~SE2CU(){}

        MX x() { return X_(0, all); };
        MX y() { return X_(1, all); };
        MX o() { return X_(2, all); };
        MX xy() { return casadi::MX::vertcat({x(), y()}); };

        //Symbolic Vars - to be used in ODE's
        SX x_ode{SX::sym("x")}; // Cartesian coordinate
        SX y_ode{SX::sym("y")}; // Cartesian coordinate
        SX o_ode{SX::sym("o")}; // Theta angle
        SX v_ode{SX::sym("v")}; // Speed
        SX p_ode{SX::sym("p")}; // Steering angle (phi)
        SX X_ode() override { return SX::vertcat({x_ode, y_ode, o_ode, v_ode, p_ode}); };
        SX XY_ode() override { return SX::vertcat({x_ode, y_ode}); };
        SX get_weights() const override {
            return SX::vertcat({w_x, w_y, w_o, w_v, w_p});
        }
        SX get_std_values() const override {
            return SX::vertcat({x_std, y_std, o_std, v_std, p_std});
        }

        std::shared_ptr<State> clone() const override;

    };

}


#endif //MROPT_SE2CU_H
