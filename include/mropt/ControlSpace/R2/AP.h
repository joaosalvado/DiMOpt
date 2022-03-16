//
// Created by ohmy on 2022-03-14.
//

#ifndef MROPT_AP_H
#define MROPT_AP_H
#pragma once

#include "../Control.hpp"
using namespace casadi;

namespace mropt::ControlSpace {
    class AP : public Control {
        double w_u_a = 100;
        double w_u_p = 100;//200
        double u_a_std = 0.0;
        double u_p_std = 0.0;
        double bound_u_a = 1.0;
        double bound_u_p = 1.0;
    public:
        AP() : Control() {
            nu_ = 2; // amount of control
            lb_ = {-bound_u_a, -bound_u_p}; // lower bound
            ub_ = {bound_u_a, bound_u_p}; // upper bound
        }
        ~AP();

        casadi::MX a() { return U_(0, all); };
        casadi::MX p() { return U_(1, all); };

        //Symbolic Vars - to be used in ODE's
        casadi::SX a_ode{SX::sym("a")};
        casadi::SX p_ode{SX::sym("p")};
        casadi::SX U_ode() override { return casadi::SX::vertcat({a_ode, p_ode}); };

        casadi::SX get_weights() const override {
            return SX::vertcat({w_u_a, w_u_p});
        }
        casadi::SX get_std_values() const override {
            return SX::vertcat({u_a_std, u_p_std});
        }

        AP &set_weights_std_values(std::vector<double> weight, std::vector<double> vars_std) override;

        std::shared_ptr<Control> clone() const override;


    };
}

#endif //MROPT_AP_H
