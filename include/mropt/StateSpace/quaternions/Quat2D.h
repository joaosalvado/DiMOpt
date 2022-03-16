//
// Created by ohmy on 2022-03-16.
//

#ifndef MROPT_QUAT2D_H
#define MROPT_QUAT2D_H


#include "../State.hpp"

using namespace casadi;
namespace mropt::StateSpace {

    class Quat2D : public State {
    private:
        casadi::Slice all{};
        double w_x = 0.0;
        double w_y = 0.0;
        double w_qz = 0.0;
        double w_qw = 0.0;
        double x_std = 0.0;
        double y_std = 0.0;
        double qz_std = 0.0;
        double qw_std = 0.0;
        double bound_qz = 1.0;
        double bound_qw = 1.0;
    public:

        Quat2D() : State() { nx_ = 4; }

        //int nx() override { return nx_; }
        casadi::MX x() override{ return X_(0, all); };

        casadi::MX y() override{ return X_(1, all); };

        casadi::MX o() override;

        casadi::MX xy() override { return casadi::MX::vertcat({x(), y()}); };

        //Symbolic Vars - to be used in ODE's
        casadi::SX x_ode{SX::sym("x")};
        casadi::SX y_ode{SX::sym("y")};
        casadi::SX qz_ode{SX::sym("qz")};
        casadi::SX qw_ode{SX::sym("qz")};

        casadi::SX X_ode() override { return casadi::SX::vertcat({x_ode, y_ode, qz_ode, qw_ode}); };

        casadi::SX XY_ode() override { return casadi::SX::vertcat({x_ode, y_ode}); };

        casadi::SX get_weights() const override {
            return SX::vertcat({w_x, w_y, w_qz, w_qw});
        }

        casadi::SX get_std_values() const override {
            return SX::vertcat({x_std, y_std, qz_ode, qw_ode});
        }

        std::shared_ptr<State> clone() const override;

        void set_bounds(casadi::Opti& ocp) override;
        std::list<casadi::MX> get_constraints() override;

        void initial_guess(
                const std::vector<double> &x0,
                const std::vector<double> &xf,
                casadi::DM &X_guess) override;

        void getSE2(
                const std::vector<double> state,
                double &x, double &y, double &o) override;

         ~Quat2D();
    };
}


#endif //MROPT_QUAT2D_H
