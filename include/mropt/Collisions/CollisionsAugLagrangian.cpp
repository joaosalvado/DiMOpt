//
// Created by ohmy on 2021-07-22.
//

#include "CollisionsAugLagrangian.h"

using namespace mropt::collisions;

void CollisionsAugLagrangian::setup(
    std::vector<std::shared_ptr<mropt::Problem::Robot>> &robots_) {
  // Set robots
  robots = robots_;
  // Setup up parameters
  N = robots[0]->ss->Nx(); // Assumes all robots have same discrete steps
  R = robots.size();
  xy_slice = robots[0]->ss->xy_slice;
  iters = 0;
  // True cost augmented lagrangian
  J_al_ = std::vector<Function*>(R,nullptr);
  // Clear previous collisions
  g_col_param = std::vector<std::unordered_map<Collision,ParametricCollision, Collision::CollisionHasher>>
      (R, std::unordered_map<Collision,ParametricCollision, Collision::CollisionHasher>());
  // Initialize rho per robot
  for(auto &robot : robots){
    robot->rho = robot->ocp_->parameter(1);
    robot->ocp_->set_value(robot->rho, 0.0);
  }
  // Generate constraint for every pair of robots when sharing a polygon
  for (int r_1 = 0; r_1 < R; ++r_1) {
    auto m1 = robots[r_1]->mission_curr;   // mission robot1
    if (m1->pas.empty())
      continue; //no polygons allocated
    for (int r_2 = r_1 + 1; r_2 < R; ++r_2) {
      auto m2 = robots[r_2]->mission_curr;   //mission robot2
      if (m2->pas.empty())
        continue; //no polygons allocated
      for (int k = 1; k < N - 1; ++k) {
        const auto &pols1 = m1->pol_alloc[k];    // Allocated polygons for robot 1 at k
        const auto &pols2 = m2->pol_alloc[k];    // Allocated polygons for robot 2 at k
        // Check if robots share the same polygon
        std::vector<int> all_pols;
        all_pols.reserve(pols1.size() + pols2.size());
        all_pols.insert(all_pols.end(), pols1.begin(), pols1.end());
        all_pols.insert(all_pols.end(), pols2.begin(), pols2.end());
        std::sort(all_pols.begin(), all_pols.end());
        auto it = std::unique(all_pols.begin(), all_pols.end());
        bool colliding = true;// ( it != all_pols.end() );//TODO: check all points
        if (colliding) {
          this->generate_parametric_collision( r_1, r_2, k);
          this->generate_parametric_collision( r_2, r_1, k);
        }
      }
    }
  }
  init_parameters();
  add_constraints();
  set_J_violation();
  set_J_violation_per_robot();
  bind_convexify_functions();
  J_violation_ = J_max_;
  J_real_ = J_sum_;
}

void CollisionsAugLagrangian::generate_parametric_collision( int r1, int r2, int k) {
  Collision col_id{r1, r2, k};
  // Parametric Collision
  ParametricCollision pcol{};
  pcol.col = col_id;
  pcol.x2_param = robots[r1]->ocp_->parameter(2, 1); // xy
  pcol.z12 = robots[r1]->ocp_->parameter(1);
  pcol.mu12 = robots[r1]->ocp_->parameter(1);
  const auto &x1 = robots[r1]->ss->X()(xy_slice, k);
  const auto &total_safe_dist
      = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
  pcol.g12 = col_.f(x1, pcol.x2_param) + total_safe_dist * total_safe_dist;
  pcol.aug_lag = pcol.mu12 * (-pcol.g12 + pcol.z12)
      + (0.5 * robots[r1]->rho) * mtimes(-pcol.g12 + pcol.z12, -pcol.g12 + pcol.z12);
  //pcol.aug_lag = pcol.mu12 * (pcol.g12 );
  //pcol.aug_lag = pcol.mu12 * (pcol.g12 ) + (0.5 * rho) * mtimes(pcol.g12, pcol.g12);
  // Store y_val and z12
  if (r1 < r2) { //set with initial guess distance
    auto dist =
        col_.f(robots[r1]->X_curr(xy_slice, k), robots[r2]->X_curr(xy_slice, k))
            + total_safe_dist * total_safe_dist;
    z_val.insert({{col_id, {dist}}});
  }
  mu_val.insert({{col_id,{mu12_val0}}});

  // Generate constraint Approximation
  pcol.approx = generate_constraint(*robots[r1]->ocp_, r1, k);
  // Insert in collision map
  g_col_param[r1].insert({{col_id, std::move(pcol)}});
}

MX CollisionsAugLagrangian::get_augmented_lagrangian(int r_id) {
  MX augmented_lagrangian{0.0};
  for (const auto &pcol_entry : g_col_param[r_id]) {
    augmented_lagrangian = augmented_lagrangian + pcol_entry.second.aug_lag;
  }
  return augmented_lagrangian;
}

void CollisionsAugLagrangian::init_parameters() {
  for (int r = 0; r < R; ++r) {
    for (const auto &param_col : g_col_param[r]) {
      int r1 = param_col.first.r1; // same as r
      int r2 = param_col.first.r2;
      int k = param_col.first.k;
      robots[r1]->ocp_->set_value(param_col.second.x2_param, robots[r2]->X_curr(xy_slice, k)); // x2
      robots[r1]->ocp_->set_value(param_col.second.mu12, mu12_val0); // mu12
      robots[r1]->ocp_->set_value(param_col.second.z12, zVal(r1, r2, k)); // z
    }
  }
}

DM CollisionsAugLagrangian::zVal(int r1, int r2, int k) {
  if (r1 < r2) {
    return z_val[{r1, r2, k}];
  }
  return z_val[{r2, r1, k}];
}

void CollisionsAugLagrangian::update_multiplers(const std::vector<std::shared_ptr<OptiSol>> &solutions) {
  // Compute average consensus quantity (zVal) to be shared by robots
  for (int r1 = 0; r1 < R; ++r1) {
    for (auto &param_col1_entry : g_col_param[r1]) {
      auto r2 = param_col1_entry.first.r2; auto k = param_col1_entry.first.k;
      if(r1 < r2) {
        const auto &total_safe_dist
            = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
        auto &param_col2 = g_col_param[r2][{r2, r1, k}];
        z_val[{r1, r2, k}]
            = 0.5 * solutions[r1]->value(param_col1_entry.second.g12) +
            0.5 * solutions[r2]->value(param_col2.g12);
//        z_val[{r1, r2, k}]
//            = 0.25 * solutions[r1]->value(param_col1_entry.second.g12) +
//            0.25 * solutions[r2]->value(param_col2.g12) +
//            0.5 * (col_.f(robots[r1]->X_curr(xy_slice,k),
//            robots[r2]->X_curr(xy_slice,k)) + total_safe_dist * total_safe_dist);
//          z_val[{r1, r2, k}] =
//            col_.f(robots[r1]->X_curr(xy_slice,k),
//            robots[r2]->X_curr(xy_slice,k)) + total_safe_dist * total_safe_dist;
//        z_val[{r1, r2, k}]
//            = 0.5 * solutions[r1]->value(param_col1_entry.second.g12)
//                + 0.5 *(col_.f(robots[r1]->X_curr(xy_slice,k),
//                   robots[r2]->X_curr(xy_slice,k)) + total_safe_dist * total_safe_dist);
//          auto real_z
//            = 0.5 * solutions[r1]->value(param_col1_entry.second.g12)
//              + 0.5*( col_.f(robots[r1]->X_curr(xy_slice,k),
//            robots[r2]->X_curr(xy_slice,k)) + total_safe_dist * total_safe_dist);
//          auto real_z
//            = 0.5 * solutions[r1]->value(param_col1_entry.second.g12) +
//            0.5 * solutions[r2]->value(param_col2.g12);
//          z_val[{r1, r2, k}] = z_val[{r1, r2, k}] + c(iters) *(real_z -z_val[{r1, r2, k}]);
//        z_val[{r1, r2, k}] = real_z;
      }
    }
  }
  //Compute new multipliers and update parameters
  DM zero{0.0};
  for( int r1 = 0; r1 < R; ++r1 ){
    for (auto &param_col1_entry : g_col_param[r1]) {
      auto r2 = param_col1_entry.first.r2; auto k = param_col1_entry.first.k;
      auto &param_col1 = param_col1_entry.second;
      //rho = rho/sqrt(++k);
      //rho = 10;

//      DM new_mu12 = solutions[r1]->value(param_col1.mu12)
//          + rho*(solutions[r1]->value(param_col1.g12 ));
      DM new_mu12 = solutions[r1]->value(param_col1.mu12)
          +  rho*(-solutions[r1]->value(param_col1.g12 ) + zVal(r1, r2, k) );
//        std::cout << param_col1.g12 << std::endl;
//        std::cout << r1 << " " << r2 << " " << k << " " << new_mu12 << " " << solutions[r1]->value(param_col1.g12 ) << std::endl;
      if(new_mu12.scalar() < 0) new_mu12 = 0; //project
      mu_val[{r1,r2,k}] = new_mu12;
      robots[r1]->ocp_->set_value(param_col1.mu12, new_mu12);// update multiplies
      auto real_x2 =
          0.5*robots[r1]->ocp_->value(param_col1.x2_param) +
              0.5*robots[r2]->X_curr(xy_slice, k);
//      robots[r1]->ocp_->set_value(
//          param_col1.x2_param,
//          robots[r1]->ocp_->value(param_col1.x2_param)
//          +c(iters)*(real_x2-robots[r1]->ocp_->value(param_col1.x2_param))); // update other robot location
//      double noise = 0.1;
//      real_x2 =
//          { real_x2((int)State::POS::x).scalar() + UniformNoise(-noise , noise) ,
//            real_x2((int)State::POS::y).scalar() + UniformNoise(-noise , noise) };
      robots[r1]->ocp_->set_value(
          param_col1.x2_param, real_x2);

      robots[r1]->ocp_->set_value(
          param_col1.z12, zVal(r1, r2, k)); // update consensus distance
      //std::cout << r1 << " " << r2 << " " << k << " " << new_mu12 << " " << solutions[r1]->value(param_col1.g12 ) << std::endl;
      //Dealing with symmetries
//      if(r1>r2) {
//        std::cout << "mu12: " << mu_val[{r1, r2, k}] << " mu21: " << mu_val[{r2, r1, k}] << std::endl;
//        if (mu_val[{r1, r2, k}].scalar() != 0.0) {
//          //if (std::abs(mu_val[{r1, r2, k}].scalar() - mu_val[{r2, r1, k}].scalar()) <= 0.00001) {
//            mu_val[{r1, r2, k}] = 0.0;
//            robots[r1]->ocp_->set_value(param_col1.mu12, 0.0);// update multiplies
//          //}
//        }
//      }
    }
  }
  iters++;

}

void CollisionsAugLagrangian::add_constraints() {
  const auto &constraint_list_all_robots = get_constraints();
  for(int r = 0; r < R; ++r) {
    auto &constraint_list = constraint_list_all_robots[r];
    int g_c_num = constraint_list.size();
    if (g_c_num) {
      int dim_g = constraint_list[0].size1();// should be dim 2 (x, y)
      MX slack_g_c_t = robots[r]->ocp_->variable(dim_g, g_c_num);
      for (int g_id = 0; g_id < g_c_num; ++g_id) {
        robots[r]->ocp_->subject_to(slack_g_c_t(all, g_id) >= 0);
        robots[r]->ocp_->subject_to(constraint_list[g_id] - slack_g_c_t(all, g_id) <= 0);
      }
      robots[r]->sum_g_col = sum2(sum1(slack_g_c_t));
      robots[r]->max_g_col = mmax(slack_g_c_t);
      robots[r]->mu_col = robots[r]->ocp_->parameter(1);
      robots[r]->mu_c_0 = robots[r]->mu_c_0_init;
      robots[r]->ocp_->set_value(robots[r]->mu_col, robots[r]->mu_c_0_init);
    }
  }
}

std::vector<std::vector<MX>> CollisionsAugLagrangian::get_constraints() {
  std::vector<std::vector<MX>> constraints
      = std::vector<std::vector<MX>>(R, std::vector<MX>{});
  for(int r = 0; r < R; ++r) {
    for (auto &param_col_entry: g_col_param[r]) {
      auto &col = param_col_entry.first;
      auto &param_col = param_col_entry.second;
      auto r1 = col.r1; auto r2 = col.r2; auto k = col.k;
      auto f_ap = param_col.approx.f_ap;//func. approximation
      const auto &xy_sym_r1 = robots[r1]->ss->X()(xy_slice, k);
      const auto &total_safe_dist
          = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
      constraints[r1].push_back(
          (*f_ap)({xy_sym_r1})[0]+ total_safe_dist*total_safe_dist
      );
    }
  }
  return constraints;
}


void CollisionsAugLagrangian::set_J_violation(){
  MX g_sum{0.0};
  MX g_max{0.0};
  MX zero{0.0};
  for( int r1 = 0; r1 < R; ++r1 ){
    for (auto &param_col1_entry : g_col_param[r1]) {
      auto r2 = param_col1_entry.first.r2;
      auto k = param_col1_entry.first.k;
      if(r1 < r2) {
        const auto &xy_sym_r1 = robots[r1]->ss->X()(xy_slice, k);
        const auto &xy_sym_r2 = robots[r2]->ss->X()(xy_slice, k);
        const auto &total_safe_dist
            = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
        const auto &g_col_r12k = col_.f(xy_sym_r1, xy_sym_r2)
            + total_safe_dist * total_safe_dist;
        g_sum = g_sum + MX::mmax(MX::vertcat({g_col_r12k, zero}));
        g_max = MX::mmax(MX::vertcat({g_max, MX::mmax(MX::vertcat({g_col_r12k, zero}))}));
      }
    }
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

void CollisionsAugLagrangian::set_J_violation_per_robot(){
  MX zero{0.0};
  for( int r1 = 0; r1 < R; ++r1 ){
    MX g_sum{0.0};
    MX g_max{0.0};
    for (auto &param_col1_entry : g_col_param[r1]) {
      auto r2 = param_col1_entry.first.r2;
      auto k = param_col1_entry.first.k;
      const auto &xy_sym_r1 = robots[r1]->ss->X()(xy_slice, k);
      const auto &xy_sym_r2 = param_col1_entry.second.x2_param;
      robots[r1]->x2param = MX::vertcat({robots[r1]->x2param , xy_sym_r2});
      const auto &total_safe_dist
          = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
      const auto &g_col_r12k = col_.f(xy_sym_r1, xy_sym_r2)
          + total_safe_dist * total_safe_dist;

      g_sum = g_sum + MX::mmax(MX::vertcat({g_col_r12k, zero}));
      g_max = MX::mmax(MX::vertcat({g_max, MX::mmax(MX::vertcat({g_col_r12k, zero}))}));
    }
    //robots[r1]->col_J_real_ = Function("J_sum", {robots[r1]->ss->X()}, {g_sum});
    robots[r1]->col_J_viol_ =
        Function("J_max",
                 {robots[r1]->ss->X(), robots[r1]->x2param},
                 {g_max});//TODO: this probably should be g_sum
    robots[r1]->col_J_real_ = g_sum;
//    robots[r1]->col_J_viol_ = g_max;
  }
}

casadi::DM CollisionsAugLagrangian::J_viol(const std::vector<casadi::DM> &x_mr)
{
  const auto &result = J_violation_(x_mr);
  return result[0];
}

casadi::DM CollisionsAugLagrangian::J_real(const std::vector<casadi::DM> &x_mr)
{
  const auto &result = J_real_(x_mr);
  return result[0];
}

void CollisionsAugLagrangian::set_J_aug_lagrangian(const std::vector<DM> &X_mr){
  for(int r1 = 0; r1 < R; ++r1){
    MX aug_lag{0.0};
    if(J_al_[r1] != nullptr) delete J_al_[r1];
    for(const auto &param_col_entry : g_col_param[r1]){
      int r2 = param_col_entry.first.r2; int k = param_col_entry.first.k;
      const auto &x1_sym = robots[r1]->ss->X()(xy_slice, k);
      const auto &x2_const = X_mr[r2](xy_slice,k);
      const auto &total_safe_dist
          = robots[r1]->shape->get_safety_radius() + robots[r2]->shape->get_safety_radius();
      MX g12 = col_.f(x1_sym, x2_const) + total_safe_dist * total_safe_dist;
      aug_lag = aug_lag + mu_val[{r1,r2,k}] * (-g12 + zVal(r1,r2,k)) +
          (0.5 * robots[r1]->rho) * mtimes(-g12 + zVal(r1,r2,k), -g12 + zVal(r1,r2,k));
      //+ mu_val[{r1,r2,k}] * (g12);
//      aug_lag = aug_lag + mu_val[{r1,r2,k}] * (g12);
    }
    J_al_[r1] = new Function("J_al", {robots[r1]->ss->X(), robots[r1]->rho}, {aug_lag});
  }
}

void CollisionsAugLagrangian::debug(const std::vector<std::shared_ptr<OptiSol>> &solutions){
  std::cout << "k \t\t mu12 \t\t mu21 \t\t z \t\t g12 \t\t g21 \t\t g_real" << std::endl;
  DM sum_g12{0.0}, sum_g21{0.0}, sum_z{0.0}, sum_greal{0.0};
  for(const auto& entry : z_val){

    auto &col = entry.first;
    const auto &col_inv = Collision{col.r2, col.r1, col.k};
    const auto &mu12 = mu_val[col].scalar();
    const auto &mu21 = mu_val[col_inv].scalar();
    //if( mu12 == 0 && mu21 == 0) continue;

    auto &z = entry.second;
    const auto &total_safe_dist = robots[col.r1]->shape->get_safety_radius() + robots[col.r2]->shape->get_safety_radius();
    const auto &g12 = g_col_param[col.r1][col].g12; // + total_safe_dist * total_safe_dist;
    const auto &g21 = g_col_param[col.r2][col_inv].g12; // + total_safe_dist * total_safe_dist;
    const auto &g_real = col_.f(robots[col.r1]->X_curr(xy_slice,col.k),
                                robots[col.r2]->X_curr(xy_slice,col.k)) + total_safe_dist * total_safe_dist;

    std::cout << std::setprecision(1) << col.k << "\t\t" << mu12 << "\t\t" << mu21 << "\t\t" << z << "\t\t"
              << solutions[col.r1]->value(g12) << "\t\t" << solutions[col.r2]->value(g21) << "\t\t" << g_real << std::endl;
    sum_g12 = sum_g12 + solutions[col.r1]->value(g12);
    sum_g21 = sum_g21 + solutions[col.r2]->value(g21);
    sum_z = sum_z + z;
    sum_greal = sum_greal + g_real;
  }
  std::cout << "total: " << sum_g12 << " "
            << sum_g21 << " " << sum_z << " " << sum_greal << std::endl;
  g12_p.push_back(sum_g12.scalar());
  g21_p.push_back(sum_g21.scalar());
  z_p.push_back(sum_z.scalar());
  greal_p.push_back(sum_greal.scalar());
  plot();
}

void CollisionsAugLagrangian::plot(){

  plt::title("Consensus Sum Distances");
  plt::named_plot("g12", g12_p);
  plt::named_plot("g21", g21_p);
  plt::named_plot("z", z_p);
  plt::named_plot("greal", greal_p);
  plt::legend();
  plt::show();
}

double CollisionsAugLagrangian::c(int k){
  double beta = 1;
  double nominator = beta/(k+1);
  double denominator = 0.0;
  for(int i=0; i<=k; ++i){
    denominator = denominator + beta/(i+1);
  }
  return nominator/denominator;
}

double CollisionsAugLagrangian::UniformNoise(double start, double end){
  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double>
      distribution(start,end);
  return distribution(generator);
}
CollisionsAugLagrangian::~CollisionsAugLagrangian() {
  for(auto J_al_r : J_al_){
    if(J_al_r!= nullptr)
      delete J_al_r;
  }
}

std::shared_ptr<MX> CollisionsAugLagrangian::X0_sym(int robot_id){
  if(robots.size() < robot_id){
    std::cerr << "[COLLISIONS_DECOUPLED] Robot not added" << std::endl;
  }
  return robots[robot_id]->traj0.X0_;
}

DM CollisionsAugLagrangian::X0_num(int robot_id){
  if(robots.size() < robot_id){
    std::cerr << "[COLLISIONS_DECOUPLED] Robot not added" << std::endl;
  }
  return robots[robot_id]->X_curr;
}

void CollisionsAugLagrangian::bind_convexify_functions(){
  for( int r = 0; r < R; ++r){
    robots[r]->convexify_decoupled_collisions =
        [&,r]( ){
          return this->convexify(r);
    };
  }
}


//void CollisionsAugLagrangian::add_constraints() {
//  for(int r = 0; r < R; ++r){
//    for(auto &param_col_entry : g_col_param[r]){
//      auto &param_col = param_col_entry.second;
//      robots[r]->ocp_->subject_to(param_col.g12+col_.threshold<0);
//    }
//  }
//}
