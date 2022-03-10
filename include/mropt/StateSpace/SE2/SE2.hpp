#ifndef SE2_H
#define SE2_H
#pragma once

#include "../State.hpp"
using namespace casadi;
namespace mropt::StateSpace {

class SE2 : public State {
private:
  casadi::Slice all{};
  double w_x = 0.0;
  double w_y = 0.0;
  double w_o = 0.0;
  double x_std = 0.0;
  double y_std = 0.0;
  double o_std = 0.0;

public:
  std::shared_ptr<State> clone() const override;

  SE2() : State() { nx_ = 3; }

  int nx() override { return nx_; }
  casadi::MX x() { return X_(0, all); };
  casadi::MX y() { return X_(1, all); };
  casadi::MX o() { return X_(2, all); };
  casadi::MX xy() { return casadi::MX::vertcat({x(), y()}); };

  //Symbolic Vars - to be used in ODE's
  casadi::SX x_ode{SX::sym("x")};
  casadi::SX y_ode{SX::sym("y")};
  casadi::SX o_ode{SX::sym("o")};
  casadi::SX X_ode() override { return casadi::SX::vertcat({x_ode, y_ode, o_ode}); };
  casadi::SX XY_ode() override { return casadi::SX::vertcat({x_ode, y_ode}); };
  casadi::SX get_weights() const override {
    return SX::vertcat({w_x, w_y, w_o});
  }
  casadi::SX get_std_values() const override {
    return SX::vertcat({x_std, y_std, o_std});
  }

  SE2 &set_weights_std_values(std::vector<double> weight, std::vector<double> vars_std) override {
    w_x = weight[(int) POS::x];
    w_y = weight[(int) POS::y];
    w_o = weight[(int) POS::o];
    x_std = vars_std[(int) POS::x];
    y_std = vars_std[(int) POS::y];
    o_std = vars_std[(int) POS::o];
    return *this;
  }

  virtual ~SE2();
};
}
#endif