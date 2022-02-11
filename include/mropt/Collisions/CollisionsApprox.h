//
// Created by ohmy on 2021-06-29.
//

#ifndef MROPT_COLLISIONS_H
#define MROPT_COLLISIONS_H

#include <vector>
#include <unordered_map>
#include <casadi/casadi.hpp>
#include "mropt/Collisions/Collisions.h"
#include <mropt/Problem/Robot.hpp>
#include <boost/functional/hash.hpp>

// class forward
namespace mropt::Problem{
 class CoupledProblem;
 class DecoupledProblem;
}

using namespace casadi;
using namespace std::placeholders;
struct Fot;

namespace mropt::collisions {
class CollisionsApprox {
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
  struct Approximation {
    std::shared_ptr<Fot> fot;
    std::shared_ptr<Function> f_ap;
  };
protected:
  std::function<void(casadi::Opti &ocp)> define_problem;
  int N{0};    // Discrete time end
  int R{0};    // Amount of robots
  const Collisions &col_;
  std::vector<std::shared_ptr<mropt::Problem::Robot>> robots;
  std::unordered_map<Collision, Approximation, Collision::CollisionHasher> g_col;  // Collision constraints
  Slice xy_slice, all;
public:
  friend class mropt::Problem::CoupledProblem;
  friend class mropt::Problem::DecoupledProblem;
  explicit CollisionsApprox(
      const Collisions &col) : col_(col) {
    define_problem = [&](casadi::Opti &ocp) { return l1_penalty(ocp); };
    //define_problem = std::bind(&CollisionsApprox::l1_penalti, this, _1);
  }
  virtual ~CollisionsApprox() = default;

  void setup(Opti &ocp, std::vector<std::shared_ptr<mropt::Problem::Robot>> &robots);
  std::vector<MX> get_constraints();
  void set_J_violation();
  DM J_viol(const std::vector<casadi::DM> &x_mr);
  DM J_real(const std::vector<casadi::DM> &x_mr);
  virtual void convexify(casadi::Opti &ocp) = 0;
  virtual void generate_constraint(casadi::Opti &ocp, int r1, int r2, int k) = 0;

  void l1_penalty(casadi::Opti &ocp);
protected:

  std::shared_ptr<MX> X0_sym(int robot_id);
  DM X0_num(int robot_id);

  DM safe_dist(int robot_id);
  // problem definiton
  Function J_sum_;
  Function J_max_;
  Function J_violation_;
  Function J_real_;
  MX mu;
  DM mu_0{10};
  const DM mu_0_init{10};
  MX sum_g, max_g;
  //Iterative costs
  DM true_cost{0.0}, prev_true_cost{0.0}, model_cost{0.0}, violation{0.0};
};
}

#endif //MROPT_COLLISIONS_H
