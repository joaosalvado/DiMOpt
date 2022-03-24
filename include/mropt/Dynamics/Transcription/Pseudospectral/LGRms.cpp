//
// Created by ohmy on 2022-03-24.
//

#include "LGRms.h"

using namespace mropt::Dynamics;



void LGRms::create_LGR_params(int degree){
    tau = casadi::collocation_points(degree, "legendre");
    casadi::collocation_coeff(tau.get_elements(), D, E, w);
}

int LGRms::computeN(int initial_N) {
    initial_N = initial_N + n/2;
    initial_N = initial_N - ( initial_N % n );
    return initial_N +1;
}

void LGRms::set_J(){
    cost_->_J = J_;
    MX params = MX::vertcat({t0, tf});
    cost_->J_ = Function("J", { this->cost_->state_space_.X(),  this->cost_->control_space_.U(), params}, {this->cost_->_J});
}

std::vector<MX> LGRms::get_constraints(casadi::Opti &ocp) {
    std::vector<MX> g;
    auto h = (this->tf - this->t0) / (N-1); // N-1 before
    J_ = 0;
    int nu = ode_->control_space_->U().size1();

    for (int k = 0; k < N-1; k+=n) {
        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
        auto o = ode_->state_space_->X()(all,Slice(k+1, k+n+1));
        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));

        // Defect constraints
        auto x_dot_ = (*(ode_approx_->fapprox(k)))({{o},
                                                    {ut}});
        auto x_dot = MX::vertcat(x_dot_);
        auto F = mtimes (h , x_dot);
        auto g_d = mtimes(transpose(D), transpose(xo)) - transpose(F);


        // Controls equal
        for(int u_k = 0; u_k < n-1; ++u_k){
            ocp.subject_to(ut(all, u_k) - ut(all, u_k+1) == 0);
        }

        // Shooting gap
        auto X_end = mtimes(xo,E);
        auto g_s = ode_->state_space_->X()(all, k+n+1) - X_end;
        //ocp.subject_to(g_s == 0);
        g.push_back(MX::vertcat({MX::vec(g_d),g_s}));



        // Cost
        MX utt;
//        if (k == N -n) {
//            utt = MX::horzcat(
//                    {
//                            ode_->control_space_->U()(all,Slice(k, k+n)),
//                            MX::zeros(nu, 1)
//                    });
//        } else {
//            utt = ode_->control_space_->U()(all,Slice(k, k+n+1));
//        }


        auto l_k_ = this->cost_->l_({{o}, {ut}});
        auto l_k = MX::vertcat(l_k_);
        J_ += h * mtimes(l_k, w); // quadrature
    }

    return g;
}

void LGRms::set_J_real() {
    MX g_sum{0.0};
    MX g_max{0.0};
    auto h = (this->tf - 0) / (N-1);
    for (int k = 0; k < N-1; k+=n) {
        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
        auto o = ode_->state_space_->X()(all,Slice(k+1, k+n+1));
        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));

        // Defect constraints
        auto x_dot_ = ode_->f()({{o},{ut}});
        auto x_dot = MX::vertcat(x_dot_);
        auto F = h * x_dot;
        auto g_d = mtimes(transpose(D), transpose(xo)) - transpose(F);

        // Shooting gap
        auto X_end = mtimes(xo,E);
        auto g_s = ode_->state_space_->X()(all, k+n+1) - X_end;

        g_sum = g_sum + sum2(sum1(MX::abs(g_d))) + sum2(sum1(MX::abs(g_s)));
        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_d))}));
        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_s))}));
//        g_sum = g_sum + sum2(sum1(MX::abs(g_s))) ;
//        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_s))}));
    }
    J_max_ = Function("J_max", {ode_->state_space_->X(), ode_->control_space_->U(), tf},
                      {g_max * N / (tf*10)}); // TODO: put it back to g_max

    J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum/10});
}
