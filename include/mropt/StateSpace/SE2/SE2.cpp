#include "SE2.hpp"  

using namespace mropt::StateSpace;
	
SE2::~SE2()
{
	

}
std::shared_ptr<State> SE2::clone() const {
  return std::make_shared<SE2>(*this);
}

void SE2::initial_guess(
        const std::vector<double> &x0,
        const std::vector<double> &xf,
        casadi::DM &X_guess) {
    std::vector<double> x_init, y_init, o_init;
    double x_0{x0[0]}, y_0{x0[1]}, o_0{x0[2]};
    double x_f{xf[0]}, y_f{xf[1]}, o_f{x0[2]};

    std::vector<double> x_k = LinearSpacedVector(x_0, x_f, Nx());
    x_init.insert(x_init.end(), x_k.begin(), x_k.end());
    std::vector<double> y_k = LinearSpacedVector(y_0, y_f, Nx());
    y_init.insert(y_init.end(), y_k.begin(), y_k.end());
    std::vector<double> o_k = LinearSpacedVector(o_0, o_f, Nx());
    o_init.insert(o_init.end(), o_k.begin(), o_k.end());

    X_guess = casadi::DM({x_init, y_init, o_init});
}

