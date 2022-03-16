//
// Created by ohmy on 2022-03-16.
//

#include "Quat2D.h"

using namespace mropt::StateSpace;

Quat2D::~Quat2D()
{


}
std::shared_ptr<State> Quat2D::clone() const {
    return std::make_shared<Quat2D>(*this);
}

void Quat2D::initial_guess(
        const std::vector<double> &x0,
        const std::vector<double> &xf,
        casadi::DM &X_guess) {
    std::vector<double> x_init, y_init, qz_init, qw_init;
    double x_0{x0[0]}, y_0{x0[1]}, qz_0{x0[2]}, qw_0{x0[3]};
    double x_f{xf[0]}, y_f{xf[1]}, qz_f{x0[2]}, qw_f{x0[3]};

    std::vector<double> x_k = LinearSpacedVector(x_0, x_f, Nx());
    x_init.insert(x_init.end(), x_k.begin(), x_k.end());
    std::vector<double> y_k = LinearSpacedVector(y_0, y_f, Nx());
    y_init.insert(y_init.end(), y_k.begin(), y_k.end());
    std::vector<double> qz_k = LinearSpacedVector(qz_0, qz_f, Nx());
    qz_init.insert(qz_init.end(), qz_k.begin(), qz_k.end());
    std::vector<double> qw_k = LinearSpacedVector(qw_0, qw_f, Nx());
    qw_init.insert(qw_init.end(), qw_k.begin(), qw_k.end());

    X_guess = casadi::DM({x_init, y_init, qz_init, qw_init});
}


casadi::MX Quat2D::o(){
    casadi::MX O = casadi::MX::zeros(1, X_.size2());

    for(int k = 0; k < Nx(); ++k){
        O(0, k) = atan(X_(3, k) / X_(2, k));
    }

    return O;
}

void Quat2D::set_bounds(casadi::Opti &ocp) {
    // unit circle
    ocp.subject_to(X_(2, all) <= bound_qz);
    ocp.subject_to(X_(2, all) >= -bound_qz);

    ocp.subject_to(X_(3, all) <= bound_qw);
    ocp.subject_to(X_(3, all) >= -bound_qw);
}

std::list<casadi::MX> Quat2D::get_constraints(){
    // qz are qw are related by unit circle (complex numbers ij)
    std::list<casadi::MX> constraints;
    for(int k = 0; k < Nx(); k++){
        // qz^2 + qw^2 - 1 = 0
        constraints.push_back( X_(2,k)*X_(2,k) + X_(3,k)*X_(3,k) -1 );
    }
    return constraints;
}

void Quat2D::getSE2(
        const std::vector<double> state,
        double &x, double &y, double &o){
    x = state[(int)POS::x];
    y = state[(int)POS::y];
    o = std::atan2(state[3] , state[2]);
}