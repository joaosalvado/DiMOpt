//
// Created by ohmy on 2021-09-16.
//

#include "SharedData.h"
using namespace mropt::Problem;
casadi::DM SharedData::Xcurr(int r, int k){
  return DM::vertcat({ x[ k + r*(N+1) ], y[ k + r*(N+1) ] });
}
casadi::DM SharedData::Xcurr(int r){
  std::vector<double> x_, y_;
  for (int k = 0; k < N + 1; ++k){
    x_.push_back(x[ k + r*(N+1) ]);
    y_.push_back(y[ k + r*(N+1) ]);
  }
  return DM({x_,y_});
}
casadi::DM SharedData::Xprev(int r, int k){
  return DM(DM::vertcat({ x_p[ k + r*(N+1) ], y_p[ k + r*(N+1) ] }));
}
casadi::DM SharedData::Xprev(int r){
  std::vector<double> x_, y_;
  for (int k = 0; k < N + 1; ++k){
    x_.push_back(x_p[ k + r*(N+1) ]);
    y_.push_back(y_p[ k + r*(N+1) ]);
  }
  return DM({x_,y_});
}
casadi::DM SharedData::Z_val(int r, int k){
    return DM::vertcat({ z_x[ k + r*(N+1) ], z_y[ k + r*(N+1) ] });
}
casadi::DM SharedData::Z_val_p(int r, int k){
    return DM::vertcat({ z_x_p[ k + r*(N+1) ], z_y_p[ k + r*(N+1) ] });
}
casadi::DM SharedData::Mu12_val(int r, int k){
    return DM::vertcat({ mu_x[ k + r*(N+1) ], mu_y[ k + r*(N+1) ] });
}
/*double SharedData::Z_val(int r1, int r2, int k){
  if(r1 > r2){
    std::swap(r1, r2); //z is simmetric
  }
  return z[ r1*(R-1)*(N+1) + (r2-1)*(N+1) + k ];
}
double SharedData::Mu12_val(int r1, int r2, int k){
  if(r1 < r2) return mu12[ r1*(R - 1)*(N+1) + (r2 - 1)*(N+1) + k ];
  return mu12[ r1*(R - 1)*(N+1)  +  r2*(N+1)  +  k ];
}*/

void SharedData::printX(){
  for (int i = 0; i < size_x; ++i) std::cout << x[i] << " ";
  std::cout << std::endl;
}
void SharedData::printY(){
  for (int i = 0; i < size_y; ++i) std::cout << y[i] << " ";
  std::cout << std::endl;
}
void SharedData::printZ(){
    for (int i = 0; i < size_z; ++i) std::cout << z_x[i] << " ";
    for (int i = 0; i < size_z; ++i) std::cout << z_y[i] << " ";
    std::cout << std::endl;
}
void SharedData::printMu12(){
    for (int i = 0; i < size_mu12; ++i) std::cout << mu_x[i] << " ";
    for (int i = 0; i < size_mu12; ++i) std::cout << mu_y[i] << " ";
    std::cout << std::endl;
}
/*void SharedData::printZ(){
  for (int i = 0; i < size_z; ++i) std::cout << z[i] << " ";
  std::cout << std::endl;
}
void SharedData::printMu12(){
  for (int i = 0; i < size_mu12; ++i) std::cout << mu12[i] << " ";
  std::cout << std::endl;
}*/
void SharedData::printXp(){
  for (int i = 0; i < size_x; ++i) std::cout << x_p[i] << " ";
  std::cout << std::endl;
}

void SharedData::setZval(int r, int k, double x, double y){
  z_x[ r*(N+1) + k ] = x;
  z_y[ r*(N+1) + k ] = y;
}

void SharedData::setZval_p(int r, int k, double x, double y){
    z_x_p[ r*(N+1) + k ] = x;
    z_y_p[ r*(N+1) + k ] = y;
}

void SharedData::setMu12val(int r, int k, double x, double y){
    mu_x[ r*(N+1) + k ] = x;
    mu_y[ r*(N+1) + k ] = y;
}
/*void SharedData::setZval(int r1, int r2, int k, double val){
  if(r1 > r2){
    z[ r1*(R-1)*(N+1) + (r2)*(N+1) + k ] = 0.0;
    return;
  }
  z[ r1*(R-1)*(N+1) + (r2-1)*(N+1) + k ] = val;
}

void SharedData::setMu12val(int r1, int r2, int k, double val){
  if(r1 > r2){
    mu12[ r1*(R-1)*(N+1) + r2*(N+1) + k ] = val;
    return;
  } //r1 < r2
  mu12[ r1*(R-1)*(N+1) + (r2-1)*(N+1) + k ] = val;
}*/


void SharedData::initializeZandMu(const mropt::collisions::Collisions &col, int proc_id){
  //Compute once
  if(proc_id == 0) {
    for (int r1 = 0; r1 < R; ++r1) {
        for (int k = 0; k < N+1; ++k) {
          auto z_x = Xcurr(r1,k)((int) mropt::StateSpace::State::POS::x).scalar();
          auto z_y = Xcurr(r1,k)((int) mropt::StateSpace::State::POS::y).scalar();
          setZval(r1, k, z_x, z_y);
          setMu12val(r1, k, mu12_init, mu12_init);
          setZval_p(r1,k, z_x, z_y);
        }
    }
  }
  //Share z and mu
  MPI_Bcast(z_x, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  MPI_Bcast(z_y, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  MPI_Bcast(mu_x, size_mu12, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  MPI_Bcast(mu_y, size_mu12, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  MPI_Bcast(z_x_p, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  MPI_Bcast(z_y_p, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
}
/*void SharedData::initializeZandMu(const mropt::collisions::Collisions &col, int proc_id){
    //Compute once
    if(proc_id == 0) {
        for (int r1 = 0; r1 < R; ++r1) {
            for (int r2 = 0; r2 < R; ++r2) {
                for (int k = 0; k < N+1; ++k) {
                    if(r1 == r2) continue;
                    const auto &total_safe_dist
                            = getSafetyDistance(r1)+ getSafetyDistance(r2);
                    auto z_val = col.f(Xcurr(r1,k), Xcurr(r2,k)).scalar() +
                                 total_safe_dist * total_safe_dist;
                    setZval(r1, r2, k, z_val);
                    setMu12val(r1, r2, k, mu12_init);
                }
            }
        }
    }
    //Share z and mu
    MPI_Bcast(z, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    MPI_Bcast(mu12, size_mu12, MPI_DOUBLE, 0, MPI_COMM_WORLD);
}*/


void SharedData::init_sync(std::shared_ptr<DistributedRobot> & robot) {
    iters = 0;
//Share robot dimensions
  MPI_Allgather(
      robot->shape->get_safety_radius_ptr(), 1, MPI_DOUBLE,
      robot->data_shared->safe_distance, 1, MPI_DOUBLE,
      MPI_COMM_WORLD);
  // Share dynamic violations
  double dynamic_viol = robot->dynamics_violation.scalar();
    MPI_Allgather(
            &dynamic_viol, 1, MPI_DOUBLE,
            robot->data_shared->dynamic_violation, 1, MPI_DOUBLE,
            MPI_COMM_WORLD);
//Share XY initial guess for all robots
  MPI_Allgather(
      robot->X_curr((int) mropt::StateSpace::State::POS::x, all)->data(), N + 1, MPI_DOUBLE,
      robot->data_shared->x, N + 1, MPI_DOUBLE,
      MPI_COMM_WORLD);
  MPI_Allgather(
      robot->X_curr((int) mropt::StateSpace::State::POS::y, all)->data(), N + 1, MPI_DOUBLE,
      robot->data_shared->y, N + 1, MPI_DOUBLE,
      MPI_COMM_WORLD);
// Initialize z and mu with initial guess values
  robot->data_shared->initializeZandMu(robot->collisions_d->col_, robot->robot_id);
}

void SharedData::sync(std::shared_ptr<DistributedRobot> & robot){
  // Copy x_curr to x_prev
  // Note: there is a new solution
  for(int i = 0; i < size_x; ++i){
    x_p[i] = x[i];
    y_p[i] = y[i];
  }
  //Gather new solution
  MPI_Allgather(
      robot->X_curr((int) mropt::StateSpace::State::POS::x, all)->data(), N + 1, MPI_DOUBLE,
      robot->data_shared->x, N + 1, MPI_DOUBLE,
      MPI_COMM_WORLD);
  MPI_Allgather(
      robot->X_curr((int) mropt::StateSpace::State::POS::y, all)->data(), N + 1, MPI_DOUBLE,
      robot->data_shared->y, N + 1, MPI_DOUBLE,
      MPI_COMM_WORLD);
}

void SharedData::update_z_mu12(const mropt::collisions::Collisions & col, int proc_id){
    //Compute once

    if(proc_id == 0) {
        for (int r1 = 0; r1 < R; ++r1) {
                for (int k = 0; k < N + 1; ++k) {
                    auto z_x_new = Xcurr(r1,k)((int) mropt::StateSpace::State::POS::x).scalar();
                   /* auto z_x_prev = Xprev(r1,k)((int) mropt::StateSpace::State::POS::x).scalar();*/
                    auto z_y_new = Xcurr(r1,k)((int) mropt::StateSpace::State::POS::y).scalar();
                   /* auto z_y_prev = Xprev(r1,k)((int) mropt::StateSpace::State::POS::y).scalar();*/

                    auto z_x_prev = Z_val(r1,k)((int) mropt::StateSpace::State::POS::x).scalar();
                    auto z_y_prev = Z_val(r1,k)((int) mropt::StateSpace::State::POS::y).scalar();
                    auto z_x_prev_prev = Z_val_p(r1,k)((int) mropt::StateSpace::State::POS::x).scalar();
                    auto z_y_prev_prev = Z_val_p(r1,k)((int) mropt::StateSpace::State::POS::y).scalar();
                    setZval_p(r1,k,z_x_prev, z_y_prev);
                    //printZ();
                    /*setZval(r1, k, z_x_new , z_y_new );*/
                    auto Z_prev = Z_val(r1,k);
                    /*setZval(r1, k,((double)1/R)*z_x_new + ((double)(R-1)/R)*z_x_prev ,
                                     ((double)1/R)*z_y_new + ((double)(R-1)/R)*z_y_prev );*/
                    double W = 2; double b = (R-1)/R;
                    auto real_z_x = ((double)1/W)*z_x_new + ((double)(W-1)/W)*z_x_prev;
                    auto real_z_y = ((double)1/W)*z_y_new + ((double)(W-1)/W)*z_y_prev;
                    /*setZval(r1,k, real_z_x + b*(real_z_x- z_x_prev) + (b*0.5)*(z_x_prev-z_x_prev_prev),
                            real_z_y + b*(real_z_y-z_y_prev) + (b*0.5)*(z_y_prev-z_y_prev_prev));*/
                    setZval(r1,k, real_z_x + b*(z_x_new- z_x_prev),
                            real_z_y + b*(z_y_new-z_y_prev) );
                    /*setZval(r1,k, real_z_x + b*(z_x_new- z_x_prev) + (1-b)*(z_x_prev -z_x_prev_prev),
                            real_z_y + b*(z_y_new-z_y_prev) + (1-b)*(z_y_prev -z_y_prev_prev));*/
                    /*setZval(r1,k, real_z_x + (b*0.5)*(z_x_prev-z_x_prev_prev),
                            real_z_y + (b*0.5)*(z_y_prev-z_y_prev_prev));*/
                    /*setZval(r1, k,((double)1/W)*z_x_new + ((double)(W-1)/W)*z_x_prev ,
                            ((double)1/W)*z_y_new + ((double)(W-1)/W)*z_y_prev );*/
                    /*auto real_z_x = 0.5*z_x_new + 0.5*z_x_prev;
                    auto real_z_y =  0.5*z_y_new + 0.5*z_y_prev;*/
                    /*setZval(r1, k, z_x_prev + c(iters)*(real_z_x-z_x_prev),
                            z_y_prev + c(iters)*(real_z_y-z_y_prev) );*/
                    /*setZval(r1, k,(1/3)*z_x_new + (2/3)*z_x_prev ,
                            (1/3)*z_y_new + (2/3)*z_y_prev );*/
                    /*auto new_mu12 = Mu12_val(r1, k);*/
                    /*auto new_mu12 = Mu12_val(r1, k) +
                                    (*rho) * (Xcurr(r1,k) - Z_val(r1, k));*/
                    auto new_mu12 = Mu12_val(r1, k) +
                               (Xcurr(r1, k) - Z_val(r1, k));
                    /*auto new_mu12 = Mu12_val(r1, k) +
                               (Z_val(r1, k) - Z_prev(r1, k));*/
                    auto mu12_x_new = new_mu12((int) mropt::StateSpace::State::POS::x).scalar();
                    auto mu12_y_new = new_mu12((int) mropt::StateSpace::State::POS::y).scalar();
                    auto prev_mu12 = Mu12_val(r1, k);
                    auto mu12_x_prev = prev_mu12((int) mropt::StateSpace::State::POS::x).scalar();
                    auto mu12_y_prev = prev_mu12((int) mropt::StateSpace::State::POS::y).scalar();
                    /*setMu12val(r1,k,
                              mu12_x_prev + c(iters)*(mu12_x_new-mu12_x_prev) ,
                               mu12_y_prev + c(iters)*(mu12_y_new-mu12_y_prev));*/
                    /*setZval(r1, k,
                            z_x_prev + c(iters)*(z_x_new-z_x_prev),
                            z_y_prev + c(iters)*(z_y_new-z_y_prev));*/
                    setMu12val(r1, k, mu12_x_new, mu12_y_new );
                   /* setZval(r1, k, z_x_new + (1/(*rho))*mu12_x_prev, z_y_new +(1/(*rho))*mu12_y_prev);*/
                    //setZval(r1, k, z_x_new , z_y_new );
                    //setZval(r1, k, 0.5*z_x_new + 0.5*z_x_prev, 0.5*z_y_new + 0.5*z_y_prev);

                }
            }
    }
    //Share z and mu
    MPI_Bcast(z_x, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    MPI_Bcast(z_y, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    MPI_Bcast(mu_x, size_mu12, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    MPI_Bcast(mu_y, size_mu12, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    iters++;
}
/*void SharedData::update_z_mu12(const mropt::collisions::Collisions & col, int proc_id){
  //Compute once
  if(proc_id == 0) {
    for (int r1 = 0; r1 < R; ++r1) {
      for (int r2 = 0; r2 < R; ++r2) {
        for (int k = 0; k < N + 1; ++k) {
          if (r1 == r2)
            continue;
          const auto &total_safe_dist
              = getSafetyDistance(r1) + getSafetyDistance(r2);
          const auto &g12
              = col.f(Xcurr(r1, k), Xprev(r2, k)).scalar() + total_safe_dist * total_safe_dist;
          const auto &g21
              = col.f(Xcurr(r2, k), Xprev(r1, k)).scalar() + total_safe_dist * total_safe_dist;
          *//*auto z_val = 0.5 * g12 + 0.5 * g21;*//*
          auto z_val = col.f(Xcurr(r1, k), Xcurr(r2, k)).scalar()
                  + total_safe_dist * total_safe_dist;
          setZval(r1, r2, k, z_val);
          auto new_mu12 = Mu12_val(r1, r2, k) +
              (*rho) * (-g12 + Z_val(r1, r2, k));
          setMu12val(r1, r2, k, new_mu12);
        }
      }
    }
  }
  //Share z and mu
  MPI_Bcast(z, size_z, MPI_DOUBLE, 0, MPI_COMM_WORLD);
  MPI_Bcast(mu12, size_mu12, MPI_DOUBLE, 0, MPI_COMM_WORLD);
}*/
double SharedData::c(int k){
    double beta = 1;
    double nominator = beta/(k+1);
    double denominator = 0.0;
    for(int i=0; i<=k; ++i){
        denominator = denominator + beta/(i+1);
    }
    return nominator/denominator;
}

double SharedData::dynamicsViolation(){
    double max_violation{0.0};
    for(int r = 0; r < R; r++){
        max_violation
            = std::max(
                    max_violation,
                    this->dynamic_violation[r]);
    }
    return max_violation;
}

double SharedData::sumColViolation(const mropt::collisions::Collisions &col){
  double g_sum{0.0};
  for (int r1 = 0; r1 < R; ++r1) {
    for (int r2 = 0; r2 < R; ++r2) {
      for (int k = 0; k < N + 1; ++k) {
        if (r1 == r2)
          continue;
        const auto &total_safe_dist
            = getSafetyDistance(r1) + getSafetyDistance(r2);
        const auto &g_real
            = col.f(Xcurr(r1, k), Xcurr(r2, k)).scalar() + total_safe_dist * total_safe_dist;
        g_sum = g_sum + std::max(g_real, 0.0);
      }
    }
  }
  return g_sum;
}

double SharedData::maxColViolation(const mropt::collisions::Collisions &col){
  double g_max{0.0};
  for (int r1 = 0; r1 < R; ++r1) {
    for (int r2 = 0; r2 < R; ++r2) {
      for (int k = 0; k < N + 1; ++k) {
        if (r1 == r2)
          continue;
        const auto &total_safe_dist
            = getSafetyDistance(r1) + getSafetyDistance(r2);
        const auto &g_real
            = col.f(Xcurr(r1, k), Xcurr(r2, k)).scalar() + total_safe_dist * total_safe_dist;
        g_max = std::max( g_max ,std::max(g_real, 0.0) );
      }
    }
  }
  return g_max;
}

double SharedData::actual_cost(std::shared_ptr<DistributedRobot> & robot) {
  DM t0f = DM({robot->p_.t0, robot->p_.tf});
  double local_cost = robot->cost->J(robot->X_curr, robot->U_curr, t0f).scalar();
  double global_sum;
  MPI_Allreduce(&local_cost, &global_sum,1,  MPI_DOUBLE, MPI_SUM,
                MPI_COMM_WORLD);
  return global_sum;
}

void SharedData::plot(std::shared_ptr<DistributedRobot> & robot, double solve_time_r) {
    if(robot->robot_id == 0) {
        double g12_{0.0};
        double g21_{0.0};
        double z_{0.0};
        double mu12_{0.0};
        double greal_{0.0};
        double min_clear{INT_MAX};
        for (int r1 = 0; r1 < R; ++r1) {
            for (int r2 = r1 + 1; r2 < R; ++r2) {
                for (int k = 0; k < N + 1; ++k) {
                    if (r1 == r2)
                        continue;
                    const auto &total_safe_dist
                            = getSafetyDistance(r1) + getSafetyDistance(r2);
                    g12_ = g12_ + robot->collisions_d->col_.f(Xcurr(r1, k), Xprev(r2, k)).scalar() +
                           total_safe_dist * total_safe_dist;
                    g21_ = g21_ + robot->collisions_d->col_.f(Xcurr(r2, k), Xprev(r1, k)).scalar() +
                           total_safe_dist * total_safe_dist;
                    greal_ = greal_ + robot->collisions_d->col_.f(Xcurr(r1, k), Xcurr(r2, k)).scalar() +
                             total_safe_dist * total_safe_dist;
                    //z_ = z_ + Z_val(r1, r2, k);

                    auto dist = std::sqrt(-robot->collisions_d->col_.f(Xcurr(r1, k), Xcurr(r2, k)).scalar() )
                                - total_safe_dist;
                    if(dist < min_clear) min_clear = dist;
                }
            }
        }

        for (int r1 = 0; r1 < R; ++r1) {
            for (int k = 0; k < N + 1; ++k) {
                z_ = z_ + sum1(sum2(Z_val(r1, k))).scalar();
                mu12_ = mu12_ + sum1(sum2(Mu12_val(r1, k))).scalar();
            }
        }
        g12_sum.push_back(g12_);
        g21_sum.push_back(g21_);
        z_sum.push_back(z_);
        mu12_sum.push_back(mu12_);
        greal_sum.push_back(greal_);
        min_clearance.push_back(min_clear);
    }

    // solving time per robot
    auto *time = new double[R];
    MPI_Gather(&solve_time_r,1, MPI_DOUBLE, time,1,  MPI_DOUBLE, 0,
               MPI_COMM_WORLD);
    for(int r = 0; r < R; ++r){
        time_r[r].push_back(time[r]);
    }
    delete[] time;

    // Cost of the multirobot system
    DM t0f = DM({robot->p_.t0, robot->p_.tf});
    double local_cost = robot->cost->J(robot->X_curr, robot->U_curr, t0f).scalar();
    auto *global_sum = new double[R];
    MPI_Gather(&local_cost,1, MPI_DOUBLE, global_sum,1,  MPI_DOUBLE, 0,
               MPI_COMM_WORLD);

    if(robot->robot_id == 0) {
        double sum_mr{0.0};
        for (int r = 0; r < R; ++r) {
            cost_r[r].push_back(global_sum[r]);
            sum_mr = sum_mr + global_sum[r];
        }
        cost_mr.push_back(sum_mr);
    }

    // Trajectories of robots
    if(robot->robot_id == 0) {
        double sum_sum{0.0};
        for (int r = 0; r < R; ++r){
            double sum{0.0};
            for(int k = 0; k < N+1; k++){
                sum += sum2(sum1(Xcurr(r,k))).scalar();
            }
            traj_r[r].push_back(sum);
            sum_sum += sum;
        }
        traj_r_sum.push_back(sum_sum);
    }
    // Plot in robot/process 0
    if(robot->robot_id == 0) {
        //  plt::clf(); plt::cla(); plt::close();
        plt::subplot(4, 2, 1);
        plt::title("Consensus Sum Distances");
        plt::named_plot("g12", g12_sum);
        plt::named_plot("g21", g21_sum);
        //plt::named_plot("z", z_sum);
        plt::named_plot("greal", greal_sum);
        plt::legend();
        plt::grid(true);
        //plt::show();

        plt::subplot(4, 2, 2);
        plt::title("Trajectories");
        for(int r = 0; r < R; ++r) {
            std::stringstream ss;
            ss << "R" << r;
            auto traj_r_percent = traj_r[r];
            std::for_each(traj_r_percent.begin(), traj_r_percent.end(), [&](double &num){
                num = num/traj_r[r][0];
            });
            plt::named_plot(ss.str(), traj_r_percent);
        }
        plt::legend();

        plt::subplot(4, 2, 3);
        plt::title("Cost Mulrirobot");
//    for (int r = 0; r < R; ++r) {
//      std::string name{"robot " + std::to_string(r)};
//      plt::named_plot(name, cost_r[r]);
//    }
        plt::named_plot("MR cost", cost_mr);

        plt::subplot(4, 2, 4);
        plt::title("Minimum Clearance");
        plt::named_plot("Distance", min_clearance);

        plt::subplot(4, 2, 5);
        plt::title("Solving Time");
        for (int r = 0; r < R; ++r) {
            std::string name{"robot " + std::to_string(r)};
            plt::named_plot(name, time_r[r]);
        }

        plt::subplot(4, 2, 6);
        plt::title("z");
        plt::named_plot("z", z_sum);
        plt::legend();

        plt::subplot(4, 2, 7);
        plt::title("mu12");
        plt::named_plot("mu12", mu12_sum);
        plt::legend();

        plt::subplot(4, 2, 8);
        plt::title("Trajectories Sum");
        plt::named_plot("Sum", traj_r_sum);

        plt::legend();

        plt::show();
    }
}

/*void SharedData::plot(std::shared_ptr<DistributedRobot> & robot, double solve_time_r) {
  if(robot->robot_id == 0) {
    double g12_{0.0};
    double g21_{0.0};
    double z_{0.0};
    double greal_{0.0};
    double min_clear{INT_MAX};
    for (int r1 = 0; r1 < R; ++r1) {
      for (int r2 = r1 + 1; r2 < R; ++r2) {
        for (int k = 0; k < N + 1; ++k) {
          if (r1 == r2)
            continue;
          const auto &total_safe_dist
              = getSafetyDistance(r1) + getSafetyDistance(r2);
          g12_ = g12_ + robot->collisions_d->col_.f(Xcurr(r1, k), Xprev(r2, k)).scalar() +
              total_safe_dist * total_safe_dist;
          g21_ = g21_ + robot->collisions_d->col_.f(Xcurr(r2, k), Xprev(r1, k)).scalar() +
              total_safe_dist * total_safe_dist;
          greal_ = greal_ + robot->collisions_d->col_.f(Xcurr(r1, k), Xcurr(r2, k)).scalar() +
              total_safe_dist * total_safe_dist;
          z_ = z_ + Z_val(r1, r2, k);

          auto dist = std::sqrt(-robot->collisions_d->col_.f(Xcurr(r1, k), Xcurr(r2, k)).scalar() ) - total_safe_dist;
          if(dist < min_clear) min_clear = dist;
        }
      }
    }
    g12_sum.push_back(g12_);
    g21_sum.push_back(g21_);
    z_sum.push_back(z_);
    greal_sum.push_back(greal_);
    min_clearance.push_back(min_clear);
  }

  // solving time per robot
  auto *time = new double[R];
  MPI_Gather(&solve_time_r,1, MPI_DOUBLE, time,1,  MPI_DOUBLE, 0,
             MPI_COMM_WORLD);
  for(int r = 0; r < R; ++r){
    time_r[r].push_back(time[r]);
  }
  delete[] time;

  // Cost of the multirobot system
  DM t0f = DM({robot->p_.t0, robot->p_.tf});
  double local_cost = robot->cost->J(robot->X_curr, robot->U_curr, t0f).scalar();
  auto *global_sum = new double[R];
  MPI_Gather(&local_cost,1, MPI_DOUBLE, global_sum,1,  MPI_DOUBLE, 0,
             MPI_COMM_WORLD);

  if(robot->robot_id == 0) {
    double sum_mr{0.0};
    for (int r = 0; r < R; ++r) {
      cost_r[r].push_back(global_sum[r]);
      sum_mr = sum_mr + global_sum[r];
    }
    cost_mr.push_back(sum_mr);
  }

  // Plot in robot/process 0
  if(robot->robot_id == 0) {
    //  plt::clf(); plt::cla(); plt::close();
    plt::subplot(2, 2, 1);
    plt::title("Consensus Sum Distances");
    plt::named_plot("g12", g12_sum);
    plt::named_plot("g21", g21_sum);
    plt::named_plot("z", z_sum);
    plt::named_plot("greal", greal_sum);
    plt::legend();
    //plt::show();

    plt::subplot(2, 2, 2);
    plt::title("Cost Mulrirobot");
//    for (int r = 0; r < R; ++r) {
//      std::string name{"robot " + std::to_string(r)};
//      plt::named_plot(name, cost_r[r]);
//    }
    plt::named_plot("MR cost", cost_mr);

    plt::subplot(2, 2, 3);
    plt::title("Minimum Clearance");
    plt::named_plot("Distance", min_clearance);

    plt::subplot(2, 2, 4);
    plt::title("Solving Time");
    for (int r = 0; r < R; ++r) {
      std::string name{"robot " + std::to_string(r)};
      plt::named_plot(name, time_r[r]);
    }

    plt::show();
  }
}*/

void SharedData::getMRTrajectory(
    std::vector<std::vector<double>> &x,
    std::vector<std::vector<double>> &y,
    std::vector<std::vector<double>> &o,
    std::vector<std::vector<std::vector<double>>> &u,
    const std::shared_ptr<DistributedRobot> & robot){
  if(robot->robot_id == 0) {
      std::cout << robot->X_curr << std::endl;
    // Already have xy
    for (int r = 0; r < R; ++r) {
      x.push_back(Xcurr(r)((int) mropt::StateSpace::State::POS::x, all).get_elements());
      y.push_back(Xcurr(r)((int) mropt::StateSpace::State::POS::y, all).get_elements());
    }
  }
  // Gather theta
  auto *o_mr = new double[R * (N + 1)];
  // Gather new solution on process/robot 0
  MPI_Allgather(
      robot->X_curr((int) mropt::StateSpace::State::POS::o, all)->data(), N + 1, MPI_DOUBLE,
      o_mr, N + 1, MPI_DOUBLE, MPI_COMM_WORLD);

  if(robot->robot_id == 0) {
    for (int r = 0; r < R; ++r) {
      auto o_r = std::vector<double>(N+1);
      for (int k = 0; k < N + 1; ++k)
        o_r[k] = o_mr[ k + r*(N+1) ];
      o.push_back(o_r);
    }
  }
  delete[] o_mr;

  // Controls
  u = std::vector<std::vector<std::vector<double>>>(R,
                                                    std::vector<std::vector<double>>( robot->cs->nu(),
                                                                                      std::vector<double>(robot->cs->Nu())));
  for (int u_id = 0; u_id < robot->cs->nu(); ++u_id) {

    auto *u_mr_i = new double[R * (robot->cs->Nu())];
    MPI_Gather(
        robot->u_sol[u_id].data(),  robot->cs->Nu() , MPI_DOUBLE,
        u_mr_i, robot->cs->Nu(), MPI_DOUBLE, 0, MPI_COMM_WORLD);

    for (int r = 0; r < R; ++r) {
      for (int k = 0; k < robot->cs->Nu(); ++k) {
        u[r][u_id][k] = u_mr_i[k + r * (robot->cs->Nu())];
      }
    }
    delete[] u_mr_i;
  }
}


void SharedData::getMRTrajectory(
        std::vector<std::vector<std::vector<double>>> &x,
        std::vector<std::vector<std::vector<double>>> &u,
        const std::shared_ptr<DistributedRobot> & robot){
    // States
    x = std::vector<std::vector<std::vector<double>>>(R,
                                                      std::vector<std::vector<double>>( robot->ss->nx(),
                                                                                        std::vector<double>(robot->ss->Nx())));
    for (int x_id = 0; x_id < robot->ss->nx(); ++x_id) {

        auto *x_mr_i = new double[R * (robot->ss->Nx())];
        MPI_Gather(
                robot->xall_sol[x_id].data(),  robot->ss->Nx() , MPI_DOUBLE,
                x_mr_i, robot->ss->Nx(), MPI_DOUBLE, 0, MPI_COMM_WORLD);

        for (int r = 0; r < R; ++r) {
            for (int k = 0; k < robot->ss->Nx(); ++k) {
                x[r][x_id][k] = x_mr_i[k + r * (robot->ss->Nx())];
            }
        }
        delete[] x_mr_i;
    }

    // Controls
    u = std::vector<std::vector<std::vector<double>>>(R,
                                                      std::vector<std::vector<double>>( robot->cs->nu(),
                                                                                        std::vector<double>(robot->cs->Nu())));
    for (int u_id = 0; u_id < robot->cs->nu(); ++u_id) {

        auto *u_mr_i = new double[R * (robot->cs->Nu())];
        MPI_Gather(
                robot->u_sol[u_id].data(),  robot->cs->Nu() , MPI_DOUBLE,
                u_mr_i, robot->cs->Nu(), MPI_DOUBLE, 0, MPI_COMM_WORLD);

        for (int r = 0; r < R; ++r) {
            for (int k = 0; k < robot->cs->Nu(); ++k) {
                u[r][u_id][k] = u_mr_i[k + r * (robot->cs->Nu())];
            }
        }
        delete[] u_mr_i;
    }
}