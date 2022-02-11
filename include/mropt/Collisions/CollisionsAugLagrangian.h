//
// Created by ohmy on 2021-07-22.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_COLLISIONSAUGLAGRANGIAN_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_COLLISIONSAUGLAGRANGIAN_H

#include "mropt/Collisions/Collisions.h"
#include "mropt/Problem/Robot.hpp"
#include "casadi/casadi.hpp"

//#include <matplotlibcpp.h>
//namespace plt = matplotlibcpp;
using namespace casadi;
struct Fotd;

namespace mropt::collisions{
class CollisionsAugLagrangian {
public:
  struct Collision {
    int r1;  // robot id 1
    int r2;  // robot id 2
    int k;   // discrete time
    bool operator==(const Collision &other) const {
      return (r1 == other.r1
          && r2 == other.r2
          && k == other.k);
    }
    struct CollisionHasher {
      std::size_t operator()(const Collision &col) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, col.r1);
        boost::hash_combine(seed, col.r2);
        boost::hash_combine(seed, col.k);
        return seed;
      }
    };
  };
  // Approximation for collision quadratic constraint
  struct Approximation {
    std::shared_ptr<Fotd> fot;
    std::shared_ptr<Function> f_ap;
  };
  //Pair-wise collision parametric with respect to robot 2
  struct ParametricCollision {
    Collision col;
    Approximation approx;
    MX g12;       // parametric constraint e.g. ||x1-x2_param||_2^2
    MX x2_param;  // x2 = (x(k), y(k)) of robot 2, i.e. xy coordinate at k robot 2
    MX z12;       // variable to handle inequalities on augmented lagrangian
    MX mu12;      // multipler associated with g12 (dual variable)
    MX aug_lag;   // augmented lagrangian mu12(g12-z12) + (rho/2)||g12-z12||_2^2
  };
protected:

  int N{0};    // Discrete time end
  int R{0};    // Amount of robots
  const Collisions &col_;
  std::vector<std::unordered_map<Collision, ParametricCollision, Collision::CollisionHasher>> g_col_param;
  std::unordered_map<Collision, DM, Collision::CollisionHasher> z_val; // consensus par-wise var
  std::unordered_map<Collision, DM, Collision::CollisionHasher> mu_val;
  std::vector<std::shared_ptr<mropt::Problem::Robot>> robots;
  Slice xy_slice, all;
public:
  double rho{0.0001};
  explicit CollisionsAugLagrangian(const Collisions &col) : col_(col) {}
  virtual ~CollisionsAugLagrangian();
  void setup(std::vector<std::shared_ptr<mropt::Problem::Robot>> &robots);
  void generate_parametric_collision(int r1, int r2, int k);
  void init_parameters();
  void update_multiplers(const std::vector<std::shared_ptr<OptiSol>> &solutions);
  MX get_augmented_lagrangian(int r_id);

  // Constraint sequential convexification
  virtual void convexify(int r) = 0;
  virtual Approximation generate_constraint(casadi::Opti &ocp, int r1, int k) = 0;
  std::shared_ptr<MX> X0_sym(int robot_id);
  DM X0_num(int robot_id);
  void add_constraints();
  std::vector<std::vector<MX>> get_constraints();
  void bind_convexify_functions();

  // Constraint Violation
  Function J_sum_;
  Function J_max_;
  Function J_violation_;
  Function J_real_;
  void set_J_violation();
  void set_J_violation_per_robot();
  DM J_viol(const std::vector<casadi::DM> &x_mr);
  DM J_real(const std::vector<casadi::DM> &x_mr);

  std::vector<Function *> J_al_;//function of augmented lagrangian
  void set_J_aug_lagrangian(const std::vector<DM> &X_mr);

  // Value to converge to
  DM zVal(int r1, int r2, int k);
  DM mu12_val0{0.0};

  void debug(const std::vector<std::shared_ptr<OptiSol>> &solutions);
  void plot();
private:
  // Auxiliary Plot vars
  std::vector<double> g12_p{};
  std::vector<double> g21_p{};
  std::vector<double> z_p{};
  std::vector<double> greal_p{};

  double c(int k);
  int iters{0};
  static double UniformNoise(double start, double end);
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_COLLISIONSAUGLAGRANGIAN_H
