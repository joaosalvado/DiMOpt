//
// Created by ohmy on 2022-03-08.
//

#include "LGLms.h"

using namespace mropt::Dynamics;

//std::vector<MX> LGLms::get_constraints(casadi::Opti &ocp) {
//    std::vector<MX> g;
//    auto h = (this->tf - this->t0) / (N); // N-1 before
//    J_ = 0;
//    int nu = ode_->control_space_->U().size1();
//
//    for (int k = 0; k < N; k+=n) {
//        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
//        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
//        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));
//
//        // Defect constraints
//        auto x_dot_ = (*(ode_approx_->fapprox(k)))({{o},
//                                                    {ode_->control_space_->U()(all, k)}});
//        auto x_dot = MX::vertcat(x_dot_);
//        auto F = 0.5 * mtimes (h , x_dot);
//        auto g_d = mtimes(D(Slice(1, n + 1), all), transpose(xo)) - transpose(F);
//
//        // Controls equal
//        for(int u_k = 0; u_k < n-1; ++u_k){
//            ocp.subject_to(ut(all, u_k) - ut(all, u_k+1) == 0);
//        }
//
//        // Shooting gap
////        auto g_s = this->ode_->state_space_->X()(all, k + 1) - o(all, n - 1);
////
////        g.push_back(g_s);
//        g.push_back(MX::vec(g_d));
//
//        // Cost
//        MX utt;
//        if (k == N -n) {
//            utt = MX::horzcat(
//                    {
//                            ode_->control_space_->U()(all,Slice(k, k+n)),
//                            MX::zeros(nu, 1)
//                    });
//        } else {
//            utt = ode_->control_space_->U()(all,Slice(k, k+n+1));
//        }
//        utt = ode_->control_space_->U()(all,k);
//        auto l_k_ = this->cost_->l_({{xo}, {utt}});
//        auto l_k = MX::vertcat(l_k_);
//        J_ += 0.5 * h * mtimes(l_k, w); // quadrature
//    }
//
////    J_ = 0;
////    for (int k = 0; k < N; ++k) {
////        J_ += integrator(this->cost_->l_, (tf - t0) / (double) N,
////                           ode_->state_space_->X()(all, k),
////                           ode_->control_space_->U()(all, k));
////    }
//    return g;
//}

//void LGLms::set_J_real() {
//    MX g_sum{0.0};
//    MX g_max{0.0};
//    auto h = (this->tf - 0) / (N - 1);
//    for (int k = 0; k < N; k+=n) {
//        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
//        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
//        //auto ut = ode_->control_space_->U()(all,Slice(k, k+n));
//        auto ut =  ode_->control_space_->U()(all,k);
//        // Defect constraints
//        auto x_dot_ = ode_->f()({{o},{ut}});
//        auto x_dot = MX::vertcat(x_dot_);
//        auto F = 0.5 * h * x_dot;
//        auto g_d = mtimes(D(Slice(1, n + 1), all), transpose(xo)) - transpose(F);
//
//        g_sum = g_sum + sum2(sum1(MX::abs(g_d)));
//        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_d))}));
//    }
//    J_max_ = Function("J_max", {ode_->state_space_->X(), ode_->control_space_->U(), tf},
//                      {g_max * N / tf}); // TODO: put it back to g_max
//
//    J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
//}


std::vector<MX> LGLms::get_constraints(casadi::Opti &ocp) {
    std::vector<MX> g;
    auto h = (this->tf - this->t0) / (N); // N-1 before
    J_ = 0;
    int nu = ode_->control_space_->U().size1();

    for (int k = 0; k < N; k+=n) {
        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
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
//        auto g_s = this->ode_->state_space_->X()(all, k + 1) - o(all, n - 1);
//
//        g.push_back(g_s);
        g.push_back(MX::vec(g_d));

        // Cost
        MX utt;
        if (k == N -n) {
            utt = MX::horzcat(
                    {
                ode_->control_space_->U()(all,Slice(k, k+n)),
                MX::zeros(nu, 1)
                    });
        } else {
            utt = ode_->control_space_->U()(all,Slice(k, k+n+1));
        }

        auto l_k_ = this->cost_->l_({{xo}, {utt}});
        auto l_k = MX::vertcat(l_k_);
        J_ += 0.5 * h * mtimes(l_k, w); // quadrature
    }

//    J_ = 0;
//    for (int k = 0; k < N; ++k) {
//        J_ += integrator(this->cost_->l_, (tf - t0) / (double) N,
//                           ode_->state_space_->X()(all, k),
//                           ode_->control_space_->U()(all, k));
//    }
    return g;
}

void LGLms::set_J_real() {
    MX g_sum{0.0};
    MX g_max{0.0};
    auto h = (this->tf - 0) / (N);
    for (int k = 0; k < N; k+=n) {
        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));

        // Defect constraints
        auto x_dot_ = ode_->f()({{o},{ut}});
        auto x_dot = MX::vertcat(x_dot_);
        auto F = 0.5 * h * x_dot;
        auto g_d = mtimes(D(Slice(1, n + 1), all), transpose(xo)) - transpose(F);

        g_sum = g_sum + sum2(sum1(MX::abs(g_d)));
        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_d))}));
    }
    J_max_ = Function("J_max", {ode_->state_space_->X(), ode_->control_space_->U(), tf},
                      {g_max * N / tf}); // TODO: put it back to g_max

    J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
}


//void LGLms::set_J_real() {
//    MX g_sum{0.0};
//    MX g_max{0.0};
//    auto h = (this->tf - 0) / (N - 1);
//    for (int k = 0; k < N; k+=n) {
//        auto xo = ode_->state_space_->X()(all,Slice(k, k+n+1));
//        auto o = ode_->state_space_->X()(all,Slice(k, k+n));
//        auto ut = ode_->control_space_->U()(all,Slice(k, k+n));
//
//        // Defect constraints
//        auto x_dot_ = ode_->f()({{o},{ut}});
//        auto x_dot = MX::vertcat(x_dot_);
//        auto F = 0.5 * h * x_dot;
//        auto g_d = mtimes(D(Slice(1, n + 1), all), transpose(xo)) - transpose(F);
//
//        g_sum = g_sum + sum2(sum1(MX::abs(g_d)));
//        g_max = mmax(MX::vertcat({g_max, mmax(MX::abs(g_d))}));
//    }
//    J_max_ = Function("J_max", {ode_->state_space_->X(), ode_->control_space_->U(), tf},
//                      {g_max * N / tf}); // TODO: put it back to g_max
//
//    J_real_ = Function("J_real", {ode_->state_space_->X(), ode_->control_space_->U(), tf}, {g_sum});
//}


void LGLms::create_LGL_params(int degree) {
    if (degree == 1) {
        this->tau = DM({-1, 1});
        this->D = DM({
                             {-0.500000000000000, 0.500000000000000, 0},
                             {-0.500000000000000, 0.500000000000000, 0}});
        this->D = this->D(Slice(0, 2));
        this->w = DM({1, 1});

    }
    if (degree == 2) {
        this->tau = DM({-1, 0, 1});
        this->D = DM({
                             {-1.50000000000000,  2,  -0.500000000000000},
                             {-0.500000000000000, 0,  0.500000000000000},
                             {0.500000000000000,  -2, 1.50000000000000}});
        this->w = DM({0.333333333333333, 1.333333333333333, 0.333333333333333});

    }
    if (degree == 3) {
        this->tau = DM({-1, -0.447213595499958, 0.447213595499958, 1});
        this->D = DM({
                             {-3,                 4.04508497187474,  -1.54508497187474, 0.500000000000000},
                             {-0.809016994374947, 0,                 1.11803398874990,  -0.309016994374947},
                             {0.309016994374947,  -1.11803398874990, 0,                 0.809016994374947},
                             {-0.500000000000000, 1.54508497187474,  -4.04508497187474, 3}});
        this->w = DM({0.166666666666667, 0.833333333333333, 0.833333333333333, 0.166666666666667});
    }

    if (degree == 4) {
        this->tau = DM({-1, -0.654653670707977, 0, 0.654653670707977, 1});
        this->D = DM({{-5,                 6.75650248872424,  -2.66666666666667, 1.41016417794243,   -0.500000000000000},
                      {-1.24099025303098,  0,                 1.74574312188794,  -0.763762615825973, 0.259009746969017},
                      {0.375000000000000,  -1.33658457769545, 0,                 1.33658457769545,   -0.375000000000000},
                      {-0.259009746969017, 0.763762615825973, -1.74574312188794, 0,                  1.24099025303098},
                      {0.500000000000000,  -1.41016417794243, 2.66666666666667,  -6.75650248872424,  5}});
        this->w = DM({0.100000000000000, 0.544444444444444, 0.711111111111111, 0.544444444444444, 0.100000000000000});
    }

    if (degree == 5) {
        this->tau = DM({-1, -0.765055323929465, -0.285231516480645, 0.285231516480645, 0.765055323929465, 1});
        this->D = DM(
                {{-7.50000000000000,  10.1414159363197,     -4.03618727030535, 2.24468464817617,  -1.34991331419049,  0.500000000000000},
                 {-1.78636494833910,  2.22044604925031e-16, 2.52342677742946,  -1.15282815853593, 0.653547507429800,  -0.237781177984231},
                 {0.484951047853569,  -1.72125695283023,    0,                 1.75296196636787,  -0.786356672223241, 0.269700610832039},
                 {-0.269700610832039, 0.786356672223241,    -1.75296196636787, 0,                 1.72125695283023,   -0.484951047853569},
                 {0.237781177984231,  -0.653547507429800,   1.15282815853593,  -2.52342677742946, 0,                  1.78636494833910},
                 {-0.500000000000000, 1.34991331419049,     -2.24468464817617, 4.03618727030535,  -10.1414159363197,  7.50000000000000}});
        this->w = DM({0.0666666666666667, 0.378474956297847, 0.554858377035487, 0.554858377035487, 0.378474956297847,
                      0.0666666666666667});
    }
}

int LGLms::computeN(int initial_N) {
    initial_N = initial_N + n/2;
    initial_N = initial_N - ( initial_N % n );
    return initial_N;
}

void LGLms::set_J(){
   cost_->_J = J_;
    MX params = MX::vertcat({t0, tf});
    cost_->J_ = Function("J", { this->cost_->state_space_.X(),  this->cost_->control_space_.U(), params}, {this->cost_->_J});
}


