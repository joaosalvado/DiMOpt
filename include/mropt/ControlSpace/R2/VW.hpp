#ifndef VW_H
#define VW_H
#pragma once

#include "../Control.hpp"
using namespace casadi;

namespace mropt::ControlSpace {
class VW : public Control {
private:
  casadi::Slice all{};

  double w_u_v = 100;
  double w_u_w = 100;//200
  double u_v_std = 0.4;
  double u_w_std = 0.0;
  double bound_u_v = 1.0;
  double bound_u_w = 2.0;
public:
  std::shared_ptr<Control> clone() const override;
public:
  VW() : Control() {
    nu_ = 2;
    lb_ = {-bound_u_v, -bound_u_w};
    ub_ = {bound_u_v, bound_u_w};
  }
  int nu() override { return nu_; }

  casadi::MX v() { return U_(0, all); };
  casadi::MX w() { return U_(1, all); };

  //Symbolic Vars - to be used in ODE's
  casadi::SX v_ode{SX::sym("v")};
  casadi::SX w_ode{SX::sym("w")};
  casadi::SX U_ode() override { return casadi::SX::vertcat({v_ode, w_ode}); };

  casadi::SX get_weights() const override {
    return SX::vertcat({w_u_v, w_u_w});
  }
  casadi::SX get_std_values() const override {
    return SX::vertcat({u_v_std, u_w_std});
  }

  VW &set_weights_std_values(std::vector<double> weight, std::vector<double> vars_std) override {
    w_u_v = weight[0];
    w_u_w = weight[1];
    u_v_std = vars_std[0];
    u_w_std = vars_std[1];
    return *this;
  }
  ~VW();
};
}
#endif