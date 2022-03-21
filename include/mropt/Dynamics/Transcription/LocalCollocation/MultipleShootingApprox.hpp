#ifndef MULTIPLESHOOTINGAPPROX_H
#define MULTIPLESHOOTINGAPPROX_H
#pragma once

#include <casadi/casadi.hpp>
#include "mropt/Dynamics/Transcription.hpp"
#include "mropt/Dynamics/OdeApprox.hpp"

namespace mropt::Dynamics {
    class MultipleShootingApprox : public Transcription {

    private:
    public:
        explicit MultipleShootingApprox(
                const std::shared_ptr<OdeApprox> &ode_approx,
                const std::shared_ptr<mropt::cost::Cost> &cost)
                : Transcription(ode_approx, cost) {


        }

        ~MultipleShootingApprox() = default;

        void set_J() override {
            MX _J = 0;
            for (int k = 0; k < N; ++k) {
                _J = _J + integrator(cost_->l_, (tf - t0) / (double) N, this->ode_->state_space_->X()(all, k),
                                     this->ode_->control_space_->U()(all, k));
            }
            cost_->_J = _J;
            MX params = MX::vertcat({t0, tf});
            cost_->J_ = Function("J", { this->ode_->state_space_->X(),  this->ode_->control_space_->U(), params}, {_J});
        }

        std::vector<MX> get_constraints(casadi::Opti &ocp) override {
            std::vector<MX> constraints{};
            MX g_sum{0.0};
            for (int k = 0; k < N; ++k) {
                const auto &x_next = ode_->state_space_->X()(all, k) +
                                     integrator(
                                             *(ode_approx_->fapprox(k)),
                                             (1 / (double) N) * tf,
                                             ode_->state_space_->X()(all, k),
                                             ode_->control_space_->U()(all, k));
                g_sum = g_sum + sum1(MX::abs(ode_->state_space_->X()(all, k + 1) - x_next));
                constraints.push_back(ode_->state_space_->X()(all, k + 1) - x_next);
            }
            J_model_ = Function("J_model", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
            return constraints;
        }

        void set_J_real() override {
            MX g_sum{0.0};
            MX g_max{0.0};
            for (int k = 0; k < N; ++k) {
                const auto &x_next = ode_->state_space_->X()(all, k) + integrator(
                        ode_->f(),
                        (1 / (double) N) * tf,
                        ode_->state_space_->X()(all, k),
                        ode_->control_space_->U()(all, k));
                g_sum = g_sum + sum1(MX::abs(ode_->state_space_->X()(all, k + 1) - x_next));
                g_max = mmax(MX::vertcat({g_max, MX::abs(ode_->state_space_->X()(all, k + 1) - x_next)}));
            }
            J_max_ = Function("J_max", {ode_->state_space_->X(), ode_->control_space_->U(), tf},
                              {g_max * N / tf}); // TODO: put it back to g_max
            J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
        }

    private:
        std::shared_ptr<Transcription> clone(const std::shared_ptr<OdeApprox> ode_approx,
                                             const std::shared_ptr<mropt::cost::Cost> &cost) const override {
            return std::make_shared<MultipleShootingApprox>(ode_approx, cost);
        }
    };
}
#endif