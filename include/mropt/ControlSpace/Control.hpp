#ifndef CONTROL_H
#define CONTROL_H
#pragma once

#include <casadi/casadi.hpp>

namespace mropt::Problem{ class Robot;}

namespace mropt::ControlSpace {
class Control {
protected:
  friend class mropt::Problem::Robot;
  casadi::Slice all;
  int nu_{0};
  int Nu_{0};
  casadi::MX U_;
  std::vector<double> ub_, lb_;

  std::list<casadi::MX> get_constraints();
public:
  Control() = default;
  void setU(casadi::MX &&U) {
    int nu_test = U.size1();
    Nu_ = U.size2();
    if (nu_test != nu_) {
      std::cerr << "[Control] Control space requires dim " << nu_test << std::endl;
    }
    U_ = std::move(U);
  }

  casadi::MX U() { return U_; };
  int nu() { return nu_; }
  int Nu() { return Nu_; };
  virtual casadi::SX U_ode() = 0;
  //virtual casadi::MX gU() = 0;
  virtual ~Control();

  virtual casadi::SX get_weights() const = 0;
  virtual casadi::SX get_std_values() const = 0;
  virtual Control &set_weights_std_values(std::vector<double> weight, std::vector<double> vars_std) = 0;

  void set_bounds(casadi::Opti &ocp);
  void set_lower_bounds(std::vector<double> lb);
  void set_upper_bounds(std::vector<double> ub);

  virtual std::shared_ptr<Control> clone() const = 0;
};
}
#endif