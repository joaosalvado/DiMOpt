//
// Created by ohmy on 2021-09-17.
//

#ifndef MROPT_INCLUDE_MROPT_COLLISIONS_DISTRIBUTEDCOLLISIONS_H
#define MROPT_INCLUDE_MROPT_COLLISIONS_DISTRIBUTEDCOLLISIONS_H

#include "mropt/Collisions/Collisions.h"
#include "mropt/Problem/DistributedRobot.h"
#include "mropt/Problem/SharedData.h"
#include "casadi/casadi.hpp"

namespace mropt::Problem {class DistributedRobot;}
namespace mropt::Problem {class SharedData;}
struct Fotdi;

namespace mropt::collisions{
class DistributedCollisions {
public:
  struct Collision {
    int r2;  // robot id 2
    int k;   // discrete time
    bool operator==(const Collision &other) const { return (r2 == other.r2 && k == other.k); }
    struct CollisionHasher {
      std::size_t operator()(const Collision &col) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, col.r2);
        boost::hash_combine(seed, col.k);
        return seed;
      }
    };
  };
  // Approximation for collision quadratic constraint
  struct Approximation {
    std::shared_ptr<Fotdi> fot;
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

  struct consensus{
      MX z;
      MX mu;
      MX aug_lag;
  };

protected:
  std::unordered_map<Collision, ParametricCollision, Collision::CollisionHasher> g_col_param;
  std::vector<consensus> consensus_vars;
  std::shared_ptr<mropt::Problem::DistributedRobot> robot;
  std::shared_ptr<mropt::Problem::SharedData> data_shared;
  int R, N;
  Slice xy_slice, all;
  int iters;
public:
  const Collisions &col_;
  explicit DistributedCollisions(const Collisions &Col);
  virtual ~DistributedCollisions();
  void setup(int R, int N, std::shared_ptr<mropt::Problem::DistributedRobot> robot);
  void generate_parametric_collision(int r2, int k);
  void init_parameters();
  MX get_augmented_lagrangian();
  void update_multipliers();
  double c(int k);

  // Costs
  Function *J_al_;
  void set_J_aug_lagrangian(const std::vector<DM> &X_mr);

  // Constraint Violation
  // Note: only defined for process 0 or equivalently robot 0
  Function J_sum_;
  Function J_max_;
  Function J_violation_;
  Function J_real_;
  void set_J_violation_per_robot();
  void set_J_violation();
  DM J_viol(const std::vector<casadi::DM> &x_mr);
  DM J_real(const std::vector<casadi::DM> &x_mr);

  void setDataShared(const std::shared_ptr<mropt::Problem::SharedData> &DataShared);

  // SCP
  virtual void convexify() = 0;
  virtual Approximation generate_constraint(casadi::Opti &ocp, int k) = 0;
  void add_constraints();
  std::vector<MX> get_constraints();
};
}

#endif //MROPT_INCLUDE_MROPT_COLLISIONS_DISTRIBUTEDCOLLISIONS_H
