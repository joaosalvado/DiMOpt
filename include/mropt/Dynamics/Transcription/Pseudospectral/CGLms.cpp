//
// Created by ohmy on 2022-03-24.
//

#include "CGLms.h"


using namespace mropt::Dynamics;



std::vector<MX> CGLms::get_constraints(casadi::Opti &ocp) {
    std::vector<MX> g;
    auto h = (this->tf - this->t0) / (N); // N-1 before
    J_ = 0;
    int nu = ode_->control_space_->U().size1();

    int i=0;
    for (int k = 0; k < N; k+=n) {
        auto xend = W(all, i++);
        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
        auto xo = MX::horzcat({o, xend});
        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));

        // Defect constraints
        auto x_dot_ = (*(ode_approx_->fapprox(k)))({{o},
                                                    {ut}});
        auto x_dot = MX::vertcat(x_dot_);
        auto F = 0.5 * mtimes (h , x_dot);
        auto g_d = mtimes(D(Slice(1, n + 1), all), transpose(xo)) - transpose(F);

        // Controls equal
        for(int u_k = 0; u_k < n-1; ++u_k){
            ocp.subject_to(ut(all, u_k) - ut(all, u_k+1) == 0);
        }

        // Shooting gap
        auto g_s = this->ode_->state_space_->X()(all, k + n) - xend;
        ocp.subject_to(g_s == 0);
//        g.push_back(MX::vertcat({MX::vec(g_d),g_s}));
//        g.push_back(g_s);
        g.push_back(MX::vec(g_d));



        // Cost
        auto l_k_ = this->cost_->l_({{o}, {ut}});
        auto l_k = MX::vertcat(l_k_);
        J_ += 0.5 * h * mtimes(l_k, w(Slice(0,n))); // quadrature
    }


    return g;
}

void CGLms::set_J_real(casadi::Opti &ocp) {
    MX g_sum{0.0};
    MX g_max{0.0};
    auto h = (this->tf - 0) / (N);
    int nx = ode_->state_space_->X().size1();
    for (int k = 0; k < N; k+=n) {
        auto xend = ocp.variable(nx, 1);
        this->W = MX::horzcat({this->W, xend});
        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
        auto xo = MX::horzcat({o, xend});
        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));

        // Defect constraints
        auto x_dot_ = ode_->f()({{o},{ut}});
        auto x_dot = MX::vertcat(x_dot_);
        auto F = 0.5 * h * x_dot;
        auto g_d = mtimes(D(Slice(1, n + 1), all), transpose(xo)) - transpose(F);

        // Shooting gap
        auto g_s = this->ode_->state_space_->X()(all, k + n) - xend;

//        g_sum = g_sum + sum2(sum1(MX::abs(g_d))) + sum2(sum1(MX::abs(g_s)));
//        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_d))}));
//        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_s))}));
        g_sum = g_sum + sum2(sum1(MX::abs(g_d)));
        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_d))}));
    }
    J_max_ = Function("J_max", {ode_->state_space_->X(), ode_->control_space_->U(), tf},
                      {g_max * N / tf}); // TODO: put it back to g_max

    J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
}

void CGLms::create_CGL_params(int degree) {
    if (degree == 1) {
        this->tau = DM( {-1,0,1});
        this->D = DM ({
                              {-0.500000000000000, 0.500000000000000, 0},
                              {-0.500000000000000, 0.500000000000000, 0}   });
        this->D = this->D(Slice(0,2));
        this->w = DM({1,1});
    }
    if (degree == 2) {
        this->tau = DM( {-1,0,1});
        this->D = DM ({
                              {-1.50000000000000,	2.00000000000000,	-0.500000000000000},
                              {-0.500000000000000,	0,	0.500000000000000},
                              {0.500000000000000,	-2,	1.50000000000000} });
        this->w = DM({0.333333333333333,1.333333333333333,0.333333333333333});
    }
    if (degree == 3) {
        this->tau = DM( {-1,-0.500000000000000,0.500000000000000,1});
        this->D = DM ({
                              {-3.16666666666667,	4.00000000000000,	-1.33333333333333,	0.500000000000000},
                              {-1.00000000000000,	0.333333333333333,	1.00000000000000,	-0.333333333333333},
                              {0.333333333333333,	-1.00000000000000,	-0.333333333333333,	1.00000000000000},
                              {-0.500000000000000,	1.33333333333333,	-4.00000000000000,	3.16666666666667} });
        this->w = DM({0.111111111111111, 0.888888888888889, 0.888888888888889, 0.111111111111111});
    }
}

int CGLms::computeN(int initial_N) {
    initial_N = initial_N + n/2;
    initial_N = initial_N - ( initial_N % n );
    return initial_N;
}

void CGLms::set_J(){
    cost_->_J = J_;
    MX params = MX::vertcat({t0, tf});
    cost_->J_ = Function("J", { this->cost_->state_space_.X(),  this->cost_->control_space_.U(), params}, {this->cost_->_J});
}


