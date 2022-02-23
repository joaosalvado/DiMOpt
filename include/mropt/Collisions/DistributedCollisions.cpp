//
// Created by ohmy on 2021-09-17.
//

#include "DistributedCollisions.h"

using namespace mropt::collisions;

DistributedCollisions::DistributedCollisions(const Collisions &Col) : col_(Col) {}
DistributedCollisions::~DistributedCollisions() {
}

void DistributedCollisions::setup(
    int R_, int N_,
    std::shared_ptr<mropt::Problem::DistributedRobot> robot_){
  // Setup up parameters
  N = N_; R = R_; robot = robot_;
  xy_slice = robot->ss->xy_slice;
  iters = 0;
  // True cost augmented lagrangian
  J_al_ = nullptr;
  // Clear previous collisions
  g_col_param.clear();
  // Initialize rho
  robot->rho = robot->ocp_->parameter(1);
  robot->ocp_->set_value(robot->rho, *data_shared->rho);
  // Generate constraint for every pair of robots when sharing a polygon
  for (int r_2 = 0; r_2 < R; ++r_2) {
    for (int k = 1; k < N - 1; ++k) {//TODO: test for k < N
      if(r_2 == robot->robot_id) continue; //skip self-collisions
      this->generate_parametric_collision( r_2, k);
    }
  }

  init_parameters();
  add_constraints();
    std::cout << "here 7" << std::endl;
//  if(robot->robot_id == 0) {
    set_J_violation();
    std::cout << "here 8" << std::endl;
    J_violation_ = J_max_;
    J_real_ = J_sum_;
//  }

  set_J_violation_per_robot();
    std::cout << "here 9" << std::endl;


}

void DistributedCollisions::setDataShared(
    const std::shared_ptr<mropt::Problem::SharedData> &DataShared) {
  data_shared = DataShared;
}


void DistributedCollisions::generate_parametric_collision( int r2, int k) {
  Collision col_id{ r2, k};
  // Parametric Collision
  ParametricCollision pcol{};
  pcol.col = col_id;
  pcol.x2_param = robot->ocp_->parameter(2, 1); // xy
/*  pcol.z12 = robot->ocp_->parameter(1);
  pcol.mu12 = robot->ocp_->parameter(1);*/
/*  const auto &x1 = robot->ss->X()(xy_slice, k);*/
/*  const auto &total_safe_dist
      = robot->shape->get_safety_radius() + data_shared->getSafetyDistance(r2);
  pcol.g12 = col_.f(x1, pcol.x2_param) + total_safe_dist * total_safe_dist;*/
/*  pcol.aug_lag = pcol.mu12 * (-pcol.g12 + pcol.z12)
      + (0.5 * robot->rho) * mtimes(-pcol.g12 + pcol.z12, -pcol.g12 + pcol.z12);*/
  // Generate constraint Approximation
  pcol.approx = generate_constraint(*robot->ocp_, k);
  // Insert in collision map
  g_col_param.insert({{col_id, std::move(pcol)}});
}
/*void DistributedCollisions::generate_parametric_collision( int r2, int k) {
    Collision col_id{ r2, k};
    // Parametric Collision
    ParametricCollision pcol{};
    pcol.col = col_id;
    pcol.x2_param = robot->ocp_->parameter(2, 1); // xy
    pcol.z12 = robot->ocp_->parameter(1);
    pcol.mu12 = robot->ocp_->parameter(1);
    const auto &x1 = robot->ss->X()(xy_slice, k);
    const auto &total_safe_dist
            = robot->shape->get_safety_radius() + data_shared->getSafetyDistance(r2);
    pcol.g12 = col_.f(x1, pcol.x2_param) + total_safe_dist * total_safe_dist;
    pcol.aug_lag = pcol.mu12 * (-pcol.g12 + pcol.z12)
                   + (0.5 * robot->rho) * mtimes(-pcol.g12 + pcol.z12, -pcol.g12 + pcol.z12);
    // Generate constraint Approximation
    pcol.approx = generate_constraint(*robot->ocp_, k);
    // Insert in collision map
    g_col_param.insert({{col_id, std::move(pcol)}});
}*/

void DistributedCollisions::init_parameters() {
    for( int k = 0; k < N+1; k++ ){
        consensus vars;
        vars.z = robot->ocp_->parameter(2,1);
        vars.mu = robot->ocp_->parameter(2, 1);
        const auto &x1 = robot->ss->X()(xy_slice, k);
       /* vars.aug_lag = mtimes( transpose(vars.mu) ,  (x1 - vars.z) )
                       + (0.5 * robot->rho) * mtimes(transpose(x1 - vars.z), x1 - vars.z);*/
        /*vars.aug_lag =
                (0.5 * robot->rho) * mtimes(transpose(x1 - vars.z + vars.mu), x1 - vars.z + vars.mu);*/
        /*vars.aug_lag =
                (0.5 * robot->rho) * mtimes(transpose(x1 - vars.z ), x1 - vars.z);*/
        vars.aug_lag =
                (0.5 * robot->rho) * mtimes(transpose(x1 - vars.z + vars.mu ),
                                            x1 - vars.z + vars.mu);
        robot->ocp_->set_value(vars.z, data_shared->Z_val(robot->robot_id, k)); // mu12
        robot->ocp_->set_value(vars.mu, data_shared->Mu12_val(robot->robot_id, k)); // z
        consensus_vars.push_back(vars);
    }
  for (const auto &param_col : g_col_param) {
    int r2 = param_col.first.r2;
    int k = param_col.first.k;
    robot->ocp_->set_value(param_col.second.x2_param, data_shared->Xcurr(r2,k)); // x2
  }
}
/*
void DistributedCollisions::init_parameters() {
    for (const auto &param_col : g_col_param) {
        int r1 = robot->robot_id; // same as r
        int r2 = param_col.first.r2;
        int k = param_col.first.k;
        robot->ocp_->set_value(param_col.second.x2_param, data_shared->Xcurr(r2,k)); // x2
        robot->ocp_->set_value(param_col.second.mu12, data_shared->Mu12_val(r1, r2, k)); // mu12
        //robot->ocp_->set_value(param_col.second.mu12, 0.0); // mu12
        robot->ocp_->set_value(param_col.second.z12, data_shared->Z_val(r1, r2, k)); // z
    }
}
*/

std::vector<MX> DistributedCollisions::get_constraints() {
  std::vector<MX> constraints{};
  for (auto &param_col_entry: g_col_param) {
    auto &col = param_col_entry.first;
    auto &param_col = param_col_entry.second;
    auto r2 = col.r2; auto k = col.k;
    auto f_ap = param_col.approx.f_ap;//func. approximation
    const auto &xy_sym_r1 = robot->ss->X()(xy_slice, k);
    const auto &total_safe_dist
        = robot->shape->get_safety_radius() + data_shared->getSafetyDistance(r2);
    constraints.push_back(
        (*f_ap)({xy_sym_r1})[0]+  (col_.threshold + total_safe_dist) * (col_.threshold + total_safe_dist)
    );
  }
  return constraints;
}

void DistributedCollisions::add_constraints() {
  const auto &constraint_list= get_constraints();
  int g_c_num = constraint_list.size();
  if (g_c_num) {
    int dim_g = constraint_list[0].size1();// should be dim 2 (x, y)
    MX slack_g_c_t = robot->ocp_->variable(dim_g, g_c_num);
    for (int g_id = 0; g_id < g_c_num; ++g_id) {
      robot->ocp_->subject_to(slack_g_c_t(all, g_id) >= 0);
      robot->ocp_->subject_to(constraint_list[g_id] - slack_g_c_t(all, g_id) <= 0);
    }
    robot->sum_g_col = sum2(sum1(slack_g_c_t));
    robot->max_g_col = mmax(slack_g_c_t);
    robot->mu_col = robot->ocp_->parameter(1);
    robot->mu_c_0 = robot->mu_c_0_init;
    robot->ocp_->set_value(robot->mu_col, robot->mu_c_0_init);
  }
//  for (auto & paramcol : g_col_param){
//    robot->ocp_->subject_to(paramcol.second.g12 < 0);
//  }
}

void DistributedCollisions::set_J_violation_per_robot() {
  MX zero{0.0};
  MX g_sum{0.0};
  MX g_max{0.0};
  robot->x2param = {};
  for (auto &param_col1_entry: g_col_param) {
    auto r2 = param_col1_entry.first.r2;
    auto k = param_col1_entry.first.k;
    const auto &xy_sym_r1 = robot->ss->X()(xy_slice, k);
    const auto &xy_sym_r2 = param_col1_entry.second.x2_param;
    robot->x2param = MX::vertcat({robot->x2param, xy_sym_r2});//TODO: this looks odd
    const auto &total_safe_dist
        = robot->shape->get_safety_radius() + data_shared->getSafetyDistance(r2);
    const auto &g_col_r12k = col_.f(xy_sym_r1, xy_sym_r2) +
            ( total_safe_dist) * ( total_safe_dist);

    g_sum = g_sum + MX::mmax(MX::vertcat({g_col_r12k, zero}));
    g_max = MX::mmax(MX::vertcat({g_max, MX::mmax(MX::vertcat({g_col_r12k, zero}))}));
  }
  //robots[r1]->col_J_real_ = Function("J_sum", {robots[r1]->ss->X()}, {g_sum});
  robot->col_J_viol_ =
      Function("J_max",
               {robot->ss->X(), robot->x2param},
               {g_max});//TODO: this probably should be g_sum
  //robot->col_J_real_ = g_sum;
  robot->col_J_real_fast_ =  Function("J_sum",
                                 {robot->ss->X(), robot->x2param},
                                 {g_sum});
//    robots[r1]->col_J_viol_ = g_max;
}

void DistributedCollisions::set_J_violation(){
  MX g_sum{0.0};
  MX g_max{0.0};
  MX zero{0.0};
  for( int r1 = 0; r1 < R; ++r1 ){
    for ( int r2 = r1+1; r2 < R; ++r2) {
      for ( int k = 1; k < N-1; ++k) {
        const auto &xy_sym_r1 =  casadi::MX::sym("xr1",2,1);
        const auto &xy_sym_r2 =  casadi::MX::sym("xr2",2,1);
          std::cout << "here 8" << std::endl;
        const auto &total_safe_dist
            = data_shared->getSafetyDistance(r1) + data_shared->getSafetyDistance(r2);
          std::cout << "here 9" << std::endl;
          std::cout << total_safe_dist << std::endl;
          std::cout << col_.threshold << std::endl;
          std::cout << col_.f(xy_sym_r1, xy_sym_r2) << std::endl;

        const auto &g_col_r12k = col_.f(xy_sym_r1, xy_sym_r2)
            + total_safe_dist * total_safe_dist;
          std::cout << "here 11" << std::endl;
        g_sum = g_sum + MX::mmax(MX::vertcat({g_col_r12k, zero}));
        g_max = MX::mmax(MX::vertcat({g_max, MX::mmax(MX::vertcat({g_col_r12k, zero}))}));
      }
    }
  }
    std::cout << "here 12" << std::endl;
  //XY vars for all robots
  std::vector<MX> X_mr{};
  for(int r = 0; r < R; ++r) {
    const auto &xyo_sym
      =  casadi::MX::sym("x_mr",robot->ss->X().size1(),robot->ss->X().size2());;
    X_mr.push_back(xyo_sym);
  }

  J_max_ = Function("J_max", {X_mr}, {g_max});
  J_sum_ = Function("J_sum", {X_mr}, {g_sum});
}

casadi::DM DistributedCollisions::J_viol(const std::vector<casadi::DM> &x_mr)
{
  const auto &result = J_violation_(x_mr);
  return result[0];
}

casadi::DM DistributedCollisions::J_real(const std::vector<casadi::DM> &x_mr)
{
  const auto &result = J_real_(x_mr);
  return result[0];
}


MX DistributedCollisions::get_augmented_lagrangian() {
  MX augmented_lagrangian{0.0};
  for (const auto &consensus : consensus_vars) {
    augmented_lagrangian = augmented_lagrangian + consensus.aug_lag;
  }
  return augmented_lagrangian;
}
/*MX DistributedCollisions::get_augmented_lagrangian() {
    MX augmented_lagrangian{0.0};
    for (const auto &pcol_entry : g_col_param) {
        augmented_lagrangian = augmented_lagrangian + pcol_entry.second.aug_lag;
    }
    return augmented_lagrangian;
}*/

void DistributedCollisions::set_J_aug_lagrangian(const std::vector<DM> &X_mr){
    MX aug_lag{0.0};
    if(J_al_ != nullptr) delete J_al_;
    for( int k = 0; k < N; ++k){
        int r1 = robot->robot_id;
        const auto &x1_sym = robot->ss->X()(xy_slice, k);
/*        aug_lag
                = aug_lag + mtimes(transpose(data_shared->Mu12_val(r1, k)) , (x1_sym - data_shared->Z_val(r1,k)) )+
                  (0.5 * robot->rho) *  mtimes(transpose(x1_sym - data_shared->Z_val(r1,k)),
                                               x1_sym - data_shared->Z_val(r1,k))
                  ;*/
/*        aug_lag
                = aug_lag +
                  (0.5 * robot->rho) *  mtimes(transpose(x1_sym - data_shared->Z_val(r1,k) + data_shared->Mu12_val(r1, k)),
                                               x1_sym - data_shared->Z_val(r1,k) + data_shared->Mu12_val(r1, k))
                ;*/
        /*aug_lag
                = aug_lag +
                  (0.5 * robot->rho) *  mtimes(transpose(x1_sym - data_shared->Z_val(r1,k)),
                                               x1_sym - data_shared->Z_val(r1,k))
                ;*/
        aug_lag
               = aug_lag +
                 (0.5 * robot->rho) *
                 mtimes(transpose(x1_sym - data_shared->Z_val(r1,k) + data_shared->Mu12_val(r1,k)),
                                              x1_sym - data_shared->Z_val(r1,k) + data_shared->Mu12_val(r1,k))
               ;

    }
    J_al_ = new Function("J_al", {robot->ss->X(), robot->rho}, {aug_lag});
}
/*void DistributedCollisions::set_J_aug_lagrangian(const std::vector<DM> &X_mr){
    MX aug_lag{0.0};
    if(J_al_ != nullptr) delete J_al_;
    for(const auto &param_col_entry : g_col_param){
        int r1 = robot->robot_id;
        int r2 = param_col_entry.first.r2; int k = param_col_entry.first.k;
        const auto &x1_sym = robot->ss->X()(xy_slice, k);
        const auto &x2_const = X_mr[r2](all,k);
        const auto &total_safe_dist
                = robot->shape->get_safety_radius() + data_shared->getSafetyDistance(r2);
        *//*MX g12 = col_.f(x1_sym, x2_const) + total_safe_dist * total_safe_dist;*//*
        MX g12 = x1_sym;
        aug_lag
                = aug_lag +
                  data_shared->Mu12_val(r1,r2,k) * (-g12 + data_shared->Z_val(r1,r2,k))+
                  (0.5 * robot->rho) *  mtimes(-g12 + data_shared->Z_val(r1,r2,k),
                                               -g12 + data_shared->Z_val(r1,r2,k));
        //+ mu_val[{r1,r2,k}] * (g12);
//      aug_lag = aug_lag + mu_val[{r1,r2,k}] * (g12);
    }
    J_al_ = new Function("J_al", {robot->ss->X(), robot->rho}, {aug_lag});

}*/

void DistributedCollisions::update_multipliers() {
  //Compute new multipliers and update parameters

    for(int k = 0; k < N+1; k++){
        auto &vars = consensus_vars[k];
       auto r1 =  robot->robot_id;
      // 0 - Update consensus penalty
      DM new_mu12 = robot->data_shared->Mu12_val(r1, k);
/*      if(new_mu12->at(0) < 0 || new_mu12->at(0) < 0 ) {
          new_mu12(0) = 0;
          new_mu12(1) = 0;
      }*/
/*      if(new_mu12->at(0) < 0) new_mu12(0) = 0; //project
      if(new_mu12->at(1) < 0) new_mu12(1) = 0; //project*/
      robot->ocp_->set_value(vars.mu, new_mu12);// update multiplies
      // 1 - Update consensus value
      robot->ocp_->set_value(vars.z, data_shared->Z_val(r1, k)); // update consensus distance
    }

    // 1 - Update other robots location
    for (auto &param_col1_entry : g_col_param) {
        auto r2 = param_col1_entry.first.r2; auto k = param_col1_entry.first.k;
        auto &param_col1 = param_col1_entry.second;
        /*auto real_x2 = data_shared->Xcurr(r2, k);*/
        auto real_x2 = data_shared->Z_val(r2,k);
        /*robot->ocp_->set_value(param_col1.x2_param,
                               0.5*data_shared->Xcurr(r2, k) +
                                    0.5*data_shared->Xprev(r2, k));*/
        /*robot->ocp_->set_value(param_col1.x2_param,
                               data_shared->Xcurr(r2, k));*/
        robot->ocp_->set_value(param_col1.x2_param,
                               data_shared->Z_val(r2, k));
/*        robot->ocp_->set_value(
                param_col1.x2_param,
                robot->ocp_->value(param_col1.x2_param)
                +c(iters)*(real_x2-robot->ocp_->value(param_col1.x2_param)));*/
    }
  iters++;
}
/*
void DistributedCollisions::update_multipliers() {
    //Compute new multipliers and update parameters
    DM zero{0.0};

    for (auto &param_col1_entry : g_col_param) {
        auto r2 = param_col1_entry.first.r2; auto k = param_col1_entry.first.k; auto r1 =  robot->robot_id;
        auto &param_col1 = param_col1_entry.second;
        // 0 - Update consensus penalty
        DM new_mu12 = robot->data_shared->Mu12_val(r1, r2, k);
        if(new_mu12.scalar() < 0) new_mu12 = 0; //project
        robot->ocp_->set_value(param_col1.mu12, new_mu12);// update multiplies
        // 1 - Update other robots location
        auto real_x2 =
                0.5*data_shared->Xprev(r2, k)+
                0.5*data_shared->Xcurr(r2, k);

*/
/*
        robot->ocp_->set_value(
             param_col1.x2_param,
              robot->ocp_->value(param_col1.x2_param)
              +c(iters)*(real_x2-robot->ocp_->value(param_col1.x2_param)));
*/
/*


        robot->ocp_->set_value(
                param_col1.x2_param, real_x2);

        // 2 - Update consensus value
        robot->ocp_->set_value(
                param_col1.z12, data_shared->Z_val(r1, r2, k)); // update consensus distance

    }
    iters++;
}
*/

double DistributedCollisions::c(int k){
    double beta = 10;
    double nominator = beta/(k+1);
    double denominator = 0.0;
    for(int i=0; i<=k; ++i){
        denominator = denominator + beta/(i+1);
    }
    return nominator/denominator;
}