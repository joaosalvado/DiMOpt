#ifndef STATE_H
#define STATE_H
#pragma once

#include <casadi/casadi.hpp>
#include <list>
namespace mropt::Problem{ class Robot;}

namespace mropt::StateSpace {
class State {
protected:
  friend class mropt::Problem::Robot;
  int nx_{0};
  int Nx_{0};
  casadi::MX X_;
  std::vector<double> lb_, ub_;

  virtual std::list<casadi::MX> get_constraints();
public:
  enum class POS { x = 0, y = 1, o = 2 };
  casadi::Slice all;
  casadi::Slice xy_slice;
  struct state {
    double x, y, o;
  };

  State() { xy_slice = casadi::Slice((int) POS::x, (int) POS::y + 1); }
  void setX(casadi::MX &&X) {
    int _nx = X.size1();
    Nx_ = X.size2();
    if (_nx != nx_) {
      std::cerr << "[State] State space requires dim " << nx_ << std::endl;
    }
    X_ = std::move(X);
  }

  casadi::MX X() { return X_; };
  int nx() {return nx_;}
  int Nx() { return Nx_; };
  virtual casadi::SX X_ode() = 0;
  virtual casadi::SX XY_ode() = 0;
  //virtual casadi::MX gX() = 0;
  virtual ~State() = default;

  virtual casadi::SX get_weights() const = 0;
  virtual casadi::SX get_std_values() const = 0;
  //virtual State &set_weights_std_values(std::vector<double> weight, std::vector<double> vars_std) = 0;

  virtual casadi::MX xy() = 0;
  virtual casadi::MX x() = 0;
  virtual casadi::MX y() = 0;
  virtual casadi::MX o() = 0;

  virtual void getSE2(
          const std::vector<double> state,
          double &x, double &y, double &o) ;

  const casadi::MX get_X_0() { return X_(all, 0); }
  const casadi::MX get_X_f() { return X_(all, Nx_ - 1); }

  virtual void set_bounds(casadi::Opti &ocp);
  virtual void initial_guess(
          const std::vector<double> &x0,
          const std::vector<double> &xf,
          casadi::DM &X_guess);

  void set_lower_bounds(std::vector<double> lb);
  void set_upper_bounds(std::vector<double> ub);

  virtual std::shared_ptr<State> clone() const = 0;

  static std::vector<double> LinearSpacedVector(double a, double b, std::size_t N);
};
}





#endif