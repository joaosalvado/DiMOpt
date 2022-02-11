//
// Created by ohmy on 2021-06-29.
//


#include "CollisionsApprox.h"

using namespace mropt::collisions;

void CollisionsApprox::setup(
    Opti &ocp,
    std::vector<std::shared_ptr<mropt::Problem::Robot>> &robots_){
  ocp.disp(std::cout);
  // Clear previous collisions
  g_col.clear();
  // Set robots
  robots = robots_;
  // Setup up parameters
  N = robots[0]->ss->Nx(); // Assumes all robots have same discrete steps
  R = robots.size();
  xy_slice = robots[0]->ss->xy_slice;
  // Generate constraint for every pair of robots when sharing a polygon
  for(int r_1 = 0; r_1 < R; ++r_1){
    auto m1 = robots[r_1]->mission_curr;   // mission robot1
//    if(m1->pas.empty()) continue; //no polygons allocated
    for(int r_2 = r_1+1; r_2 < R; ++r_2){
      auto m2 = robots[r_2]->mission_curr;   //mission robot2
//      if(m2->pas.empty()) continue; //no polygons allocated
      for(int k = 1; k < N-1; ++k){
//        const auto &pols1 = m1->pol_alloc[k];    // Allocated polygons for robot 1 at k
//        const auto &pols2 = m2->pol_alloc[k];    // Allocated polygons for robot 2 at k
//        // Check if robots share the same polygon
//        std::vector<int> all_pols; all_pols.reserve(pols1.size() + pols2.size());
//        all_pols.insert(all_pols.end(), pols1.begin(), pols1.end());
//        all_pols.insert(all_pols.end(), pols2.begin(), pols2.end());
//        std::sort( all_pols.begin(), all_pols.end() );
//        auto it = std::unique( all_pols.begin(), all_pols.end() );
        bool colliding = true; //( it != all_pols.end() );//TODO: check all points
        if( colliding ){
          generate_constraint(ocp,  r_1,  r_2, k);
        }
      }
    }
  }
  define_problem(ocp);
  set_J_violation();
  J_violation_ = J_max_;
  J_real_ = J_sum_;
}


void CollisionsApprox::l1_penalty(casadi::Opti& ocp){
  const auto &constraint_list = get_constraints();
  int g_c_num = constraint_list.size();
  if(g_c_num){
    int dim_g = constraint_list[0].size1();// should be dim 2 (x, y)
    MX slack_g_c_t = ocp.variable(dim_g, g_c_num);
    for (int g_id = 0; g_id < g_c_num; ++g_id)
    {
      ocp.subject_to(slack_g_c_t(all, g_id) >= 0);
      ocp.subject_to(constraint_list[g_id] - slack_g_c_t(all, g_id) <= 0);
    }
    sum_g = sum2(sum1(slack_g_c_t));
    max_g = mmax(slack_g_c_t);
    mu = ocp.parameter(1);
    mu_0 = mu_0_init;
    ocp.set_value(mu, mu_0_init);
  }
}




std::vector<MX> CollisionsApprox::get_constraints() {
  std::vector<MX> constraints{};
  for(auto &collision_approx : this->g_col){
    auto r1 = collision_approx.first.r1;
    auto r2 = collision_approx.first.r2;
    auto k = collision_approx.first.k;
    auto f_ap = collision_approx.second.f_ap;//func. approximation
    const auto &xy_sym_r1 = robots[r1]->ss->X()(xy_slice, k);
    const auto &xy_sym_r2 = robots[r2]->ss->X()(xy_slice, k);
    const auto &total_safe_dist = safe_dist(r1) + safe_dist(r2);
    constraints.push_back((*f_ap)({{xy_sym_r1}, {xy_sym_r2}})[0]+ total_safe_dist*total_safe_dist);
  }
  return constraints;
}


void  CollisionsApprox::set_J_violation(){
  MX g_sum{0.0};
  MX g_max{0.0};
  MX zero{0.0};
  for(auto &collision_approx : this->g_col){
    auto r1 = collision_approx.first.r1;
    auto r2 = collision_approx.first.r2;
    auto k = collision_approx.first.k;
    const auto &xy_sym_r1 = robots[r1]->ss->X()(xy_slice, k);
    const auto &xy_sym_r2 = robots[r2]->ss->X()(xy_slice, k);
    const auto &total_safe_dist = safe_dist(r1) + safe_dist(r2);
    const auto & g_col_r12k = col_.f(xy_sym_r1, xy_sym_r2) + col_.threshold + total_safe_dist*total_safe_dist ;// TODO: safe_dist might not convert to DM properly
    g_sum = g_sum + MX::mmax(MX::vertcat({g_col_r12k, zero}));
    g_max = MX::mmax( MX::vertcat({g_max,  MX::mmax( MX::vertcat({g_col_r12k, zero}) ) }) );
  }
  //XY vars for all robots
  std::vector<MX> X_mr{};
  for(int r = 0; r < R; ++r) {
      const auto &xyo_sym = robots[r]->ss->X();
      X_mr.push_back(xyo_sym);
  }
  J_max_ = Function("J_max", {X_mr}, {g_max});
  J_sum_ = Function("J_sum", {X_mr}, {g_sum});
}


std::shared_ptr<MX> CollisionsApprox::X0_sym(int robot_id){
  if(robots.size() < robot_id){
    std::cerr << "[COLLISIONSAPPROX] Robot not added" << std::endl;
  }
  return robots[robot_id]->traj0.X0_;
}

DM CollisionsApprox::X0_num(int robot_id){
  if(robots.size() < robot_id){
    std::cerr << "[COLLISIONSAPPROX] Robot not added" << std::endl;
  }
  return robots[robot_id]->X_curr;
}

DM CollisionsApprox::safe_dist(int robot_id){
  if(robots.size() < robot_id){
    std::cerr << "[COLLISIONSAPPROX] Robot not added" << std::endl;
  }
  return DM({robots[robot_id]->shape->get_safety_radius()});
}

casadi::DM CollisionsApprox::J_viol(const std::vector<casadi::DM> &x_mr)
{
  const auto &result = J_violation_(x_mr);
  return result[0];
}

casadi::DM CollisionsApprox::J_real(const std::vector<casadi::DM> &x_mr)
{
  const auto &result = J_real_(x_mr);
  return result[0];
}
