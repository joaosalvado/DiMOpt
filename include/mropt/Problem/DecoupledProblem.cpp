#include "DecoupledProblem.hpp"
#include "mropt/Problem/SharedData.h"

using namespace mropt::Problem;

void DecoupledProblem::reset_ocp() {
    // --- Problem and Solver Options
    p_opts["expand"] = true;
    //s_opts["fixed_variable_treatment"] = "make_constraint";
    Dict solver_options;
    if (solver == "qpoases") {
        s_opts["sparse"] = true;
        s_opts["schur"] = "schur";
        s_opts["print_time"] = true;
    }
    if (solver == "cplex") {
        p_opts["dump_to_file"] = true;
        p_opts["dump_filename"] = "qp_cplex.pb";
        //p_opts["qp_method"] = 8;
        //s_opts["CPX_PARAM_BARALG"] = 3;
    }
    if (solver == "ipopt") {
        s_opts["print_level"] = 0;
        s_opts["sb"] = "yes";
        p_opts["print_time"] = 0;
        s_opts["warm_start_init_point"] = "yes";
        //s_opts["fixed_variable_treatment"] = "make_constraint";
        //s_opts["mu_strategy"] = "adaptive";
        //s_opts["bound_relax_factor"] = 0;
        s_opts["linear_solver"] = "ma27";
    }

    // Decoupled set up
    robot->ocp_ = std::make_shared<Opti>();
    robot->ocp_->solver(solver, p_opts, s_opts);

    //reset();
}

DecoupledProblem &DecoupledProblem::addRobot(const std::shared_ptr<DistributedRobot> &robot_) {
    robot = robot_;
    return *this;
}

void DecoupledProblem::reset() {
    //robot->plot_ = true;
    reset_ocp();
    robot->resetQuery(*robot->ocp_);
}

void DecoupledProblem::setup() {
    robot->data_shared = std::make_shared<SharedData>(robot->p_.N, R);
    robot->collisions_d->setDataShared(robot->data_shared);

    robot->setupQuery(*robot->ocp_);

    warm_start();

    // TODO: DOIng this - Pre solve without collision constraints
    robot->ocp_->minimize(robot->J_model);
    robot->true_cost_ = [&](const DM &X, const DM &U) {
        DM result = robot->true_cost_l1penalty(X, U);
        return result;
    };


    //Solve with no collision constraints between robots
    robot->scp_d_nocol();

    // 2 - Synchronize
    robot->data_shared->sync(robot);


    if (plot_) {
        //robot->data_shared->plot(robot, solve_time_r);
        //show_stats();
//        plot_trajectories(
//               std::vector<std::shared_ptr<Dynamics::ode>>(R, robot->get_ode()));//, robot->get_ode( )});
    }

    //Share trajectories between robots
    robot->data_shared->init_sync(robot);
    robot->collisions_d->setup(R, robot->p_.N, robot);
    robot->J_model =
            robot->J_model +
            robot->collisions_d->get_augmented_lagrangian() +
            robot->mu_col * robot->sum_g_col;

    robot->ocp_->minimize(robot->J_model);

    // Clear Plot vars
    if (plot_ || debug_) robot->clear_plot_vars();
}

void DecoupledProblem::warm_start() {
    robot->ocp_->set_initial(robot->ss->X(), robot->X_curr);
    robot->ocp_->set_initial(robot->cs->U(), robot->U_curr);
}

DM DecoupledProblem::prev_actual_cost() {
    DM total_cost{0.0};
//  for (const auto &robot : robots)
//  {
//    DM t0f = DM({robot->p_.t0, robot->p_.tf});
//    total_cost = total_cost + robot->cost->J(robot->X_curr, robot->U_curr, t0f);
//  }
    return total_cost;
}

DM DecoupledProblem::actual_cost() {
    DM total_cost{0.0};
//  for (const auto &robot : robots)
//  {
//    DM t0f = DM({robot->p_.t0, robot->p_.tf});
//    total_cost = total_cost + robot->cost->J(robot->X_sol, robot->U_sol, t0f);
//  }
    return total_cost;
}


DecoupledProblem &DecoupledProblem::show_stats() {
    if (!plot_) std::cerr << "[DecoupledProblem] Call allow_plotting() before solving" << std::endl;
    robot->plot();
    return *this;
}

DecoupledProblem &DecoupledProblem::plot_trajectories(
        const std::vector<std::shared_ptr<mropt::Dynamics::ode>> &odes) {
    if (!plot_) std::cerr << "[DecoupledProblem] Call allow_plotting() before solving" << std::endl;
    if (plotter == nullptr) {
        std::cerr << "[DecoupledProblem] use set_plotter()" << std::endl;
        return *this;
    }
    if (plotter->odes.empty()) {//uninitialized
        plotter->set_models(odes);
    }

    /*std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> y;
    std::vector<std::vector<double>> o;*/
    std::vector<std::vector<std::vector<double>>> x;
    std::vector<std::vector<std::vector<double>>> u;
    robot->data_shared->getMRTrajectory(x, u, robot);
    if (robot->robot_id == 0) {
        plotter->plot_trajectory(x, u, robot->p_.tf);
    }
    return *this;
}

void DecoupledProblem::solve() {
    MPI_Barrier(MPI_COMM_WORLD); //sync point
    double setup_time{0.0};
    setup_time -= MPI_Wtime();

    reset();
    setup();
    MPI_Barrier(MPI_COMM_WORLD); //sync point
    setup_time += MPI_Wtime();
    record_curr.t_setup = setup_time;

    //admm();
    test();
    reset();
}


DecoupledProblem &DecoupledProblem::allow_plotting() {
    if (!plotter) std::cerr << "[OPTPROBLEM] provide a plotter facility" << std::endl;
    plot_ = true;
    if (!robot) std::cerr << "[OPTPROBLEM] addRobot()" << std::endl;
    robot->plot_ = true;
    return *this;
}

DecoupledProblem &DecoupledProblem::debug_mode() {
    if (!robot) std::cerr << "[OPTPROBLEM] addRobot()" << std::endl;
    robot->debug_ = true;
    this->debug_ = true;
    return *this;
}


void DecoupledProblem::test() {
    double delta_f_tol{1 * 10e-2};
    int feasible_iter{-1};
    bool found_first{false};
    double query_time{0.0}, query_time_1{0.0}, cost{0.0}, cost_1{0.0};
    total_prev_true_cost = 0.0;

    double share_time_r{0.0};
    double max_solve_time_r{0.0};
    double last_solve_time_r{0.0};
    int max_iter = 300;
    record_curr.iter_1 = max_iter;
    for (int i = 0; i < max_iter; i++) {
        // Set rho
        robot->ocp_->set_value(robot->rho, DM(*robot->data_shared->rho));

        // 0 - Set true cost function l1_penalty + augmented_lagrangian
        std::vector<DM> X_mr_prev(R);
        for (int r = 0; r < R; ++r) {
            X_mr_prev[r] = robot->data_shared->Xcurr(r);
        }

        robot->collisions_d->set_J_aug_lagrangian(X_mr_prev);
        robot->x2param_list = robot->ocp_->value(robot->x2param);

        robot->true_cost_ = [&](const DM &X, const DM &U) {
            DM result{0.0};
            result += robot->true_cost_l1penalty(X, U);
            result += robot->collisions_d->J_al_->operator()
                    (std::vector<casadi::DM>{{X},
                                             {*robot->data_shared->rho}})[0];
            //result +=     robot->mu_c_0 * robot->solution->value(robot->col_J_real_);
            result += robot->mu_c_0 *
                      robot->col_J_real_fast_(std::vector<casadi::DM>{{X},
                                                                      {robot->x2param_list}})[0];
            return result;
        };
        MPI_Barrier(MPI_COMM_WORLD); //sync point
        double solve_time_r{0.0};
        solve_time_r -= MPI_Wtime();
        max_solve_time_r -= MPI_Wtime();
        // 1 - Solving
        std::cout << "\n\n\n" << std::endl;
        //TODO: solve for robot 0 and then for robot 1  (only for debug)
        robot->scp_d();

        solve_time_r += MPI_Wtime();
        MPI_Barrier(MPI_COMM_WORLD); //sync point
        max_solve_time_r += MPI_Wtime();
        share_time_r -= MPI_Wtime();

        last_solve_time_r = solve_time_r;

        // 2 - Synchronize
        robot->data_shared->sync(robot);
        robot->data_shared->update_z_mu12(robot->collisions_d->col_, robot->robot_id);
        robot->collisions_d->update_multipliers();

        MPI_Barrier(MPI_COMM_WORLD); //sync point
        share_time_r += MPI_Wtime();

        // Checking violations and Cost improvement
        total_true_cost = robot->data_shared->actual_cost(robot);
        double cost_improvement
                = std::abs(total_prev_true_cost - total_true_cost)
                  / total_prev_true_cost;
        total_prev_true_cost = total_true_cost;
        double max_collision = robot->data_shared->maxColViolation(robot->collisions_d->col_);
        double sum_collision;
        double dynamics_viol = robot->data_shared->dynamicsViolation();


        if (plot_ || debug_) {
            sum_collision = robot->data_shared->sumColViolation(robot->collisions_d->col_);
            //show_stats();
            std::cout << "The solve time " << solve_time_r << " seconds to run." << std::endl;
            if (robot->robot_id == 0) {
                std::cout << "The share time " << share_time_r << " seconds to run." << std::endl;
                std::cout << "Collisions MAX: " << max_collision << std::endl;
                std::cout << "Collisions SUM: " << sum_collision << std::endl;
                std::cout << "Cost Improvement: " << cost_improvement << std::endl;
            }
            if (plot_) {
                //robot->data_shared->plot(robot, solve_time_r);
                //show_stats();
//                plot_trajectories(
//                        std::vector<std::shared_ptr<Dynamics::ode>>(R, robot->get_ode()));//, robot->get_ode( )});
            }
        }

        query_time = max_solve_time_r + share_time_r;// query_time + robot->ocp_query_time.count();

        // Save first feasible sol iteration
        if (max_collision <= robot->collisions_d->col_.threshold && dynamics_viol <= robot->f_tol && !found_first) {
            found_first = true;
            feasible_iter = i;
            //query_time_1 =  robot->ocp_query_time.count();
            query_time_1 = max_solve_time_r + share_time_r;
            cost_1 = robot->data_shared->actual_cost(robot);
            record_curr.iter_1 = i+1;
            // break; // Todo: comment me
        }
        // Exit condition
        if (max_collision <= robot->collisions_d->col_.threshold
            && cost_improvement <= delta_f_tol) break; //TODO: uncomment me

        total_prev_true_cost = total_true_cost;
        MPI_Barrier(MPI_COMM_WORLD); //sync point
        if(i == max_iter-1 && robot->robot_id == 0 && !found_first) record_curr.fail_status = -2;
        if(i == max_iter-1 && robot->robot_id == 0 &&  found_first) record_curr.fail_status = -3;
    }
    cost = robot->data_shared->actual_cost(robot);
    if (robot->robot_id == 0) {
        std::cout << "Query Time 1: " << query_time_1 << " iter: " << feasible_iter << std::endl;
        std::cout << "Query Time: " << max_solve_time_r + share_time_r << std::endl;
//  std::cout << "R" << proc_id_ << "Query Time 1: " << query_time_1 << " iter: " << feasible_iter << std::endl;
//  std::cout << "R" << proc_id_ << " Query Time: " << max_solve_time_r + share_time_r<< std::endl;
        std::cout << "Cost: " << cost << std::endl;

    }
    if (plot_) {
        /*auto solve_time = MPI_Wtime();*/
        //robot->data_shared->plot(robot, last_solve_time_r);
        //show_stats();
        //plot_trajectories(std::vector<std::shared_ptr<mropt::Dynamics::ode>>(R, robot->get_ode()) );
    }

        // Recording data
    if(robot->robot_id == 0) {
        record_curr.R = R;
        record_curr.t_1 = query_time_1;
        record_curr.t = query_time;
        record_curr.T = robot->p_.tf;
        record_curr.N = robot->p_.N;
        record_curr.L = robot->shape->get_safety_radius();
        record_curr.cost = cost;
        record_curr.cost_1 = cost_1;
    }
//  std::cout << "Query Time: " << query_time << std::endl;
//  std::cout << "Cost: " << actual_cost() << std::endl;
}


void DecoupledProblem::admm() {
//  double delta_f_tol{1*10e-3};
//  int feasible_iter {-1};
//  bool found_first{false};
//  double query_time{0.0}, query_time_1{0.0};
//  total_prev_true_cost = 0.0;
//  for(int i = 0; i<10; i++){
//    if(i==1){
//      for(auto &robot : robots){
//        robot->ocp_->set_value(robot->rho, DM(collisions_d->rho));
//      }
//    }
//    // Set true cost function l1_penalty + augmented_lagrangian
//    std::vector<DM> X_mr_prev{};
//    for(const auto &robot: robots) { X_mr_prev.push_back(robot->X_curr);}
//    collisions_d->set_J_aug_lagrangian(X_mr_prev);
//    for(int r = 0; r < robots.size(); ++r){
//      robots[r]->true_cost_ = [&, r](const DM& X, const DM& U){
//        return robots[r]->true_cost_l1penalty(X , U) +
//            collisions_d->J_al_[r]->operator()
//                (std::vector<casadi::DM>{{X}, {robots[r]->solution->value(robots[r]->rho)}})[0] +
//            robots[r]->mu_c_0 * robots[r]->solution->value(robots[r]->col_J_real_);
//      };
//    }
//
//    // Solve
//    std::vector<std::shared_ptr<OptiSol>> solutions;
//    for (auto &robot : robots) {
//      robot->scp();
//      solutions.push_back(robot->solution);
//    }
//
//    // Update multipliers according to newly computed solution
//    collisions_d->update_multiplers(solutions);
//
//    // Checking violations and Cost improvement
//    std::vector<DM> X_mr_curr{};
//    for(const auto &robot: robots) { X_mr_curr.push_back(robot->X_curr);}
//    total_true_cost = actual_cost();
//    double cost_improvement
//        = std::abs(total_prev_true_cost.scalar()-total_true_cost.scalar())
//            /total_prev_true_cost.scalar();
//    double max_collision = collisions_d->J_viol(X_mr_curr).scalar();
//    double sum_collision = collisions_d->J_real(X_mr_curr).scalar();
//    if(plot_) {
//      std::cout << "Collisions MAX: " << max_collision << std::endl;
//      std::cout << "Collisions SUM: " << sum_collision << std::endl;
//      std::cout << "Cost Improvement: " << cost_improvement << std::endl;
//      //collisions_d->debug(solutions);
//    }
//    // Exit condition
//    if( sum_collision == 0.0 && cost_improvement <= delta_f_tol) break;
//    total_prev_true_cost = total_true_cost;
//    // Save first feasible sol iteration
//    if(collisions_d->J_real(X_mr_curr).scalar()==0 && !found_first){
//      found_first = true;
//      feasible_iter = i;
//      for( auto &robot : robots){
//        query_time_1 = query_time_1 + robot->ocp_query_time.count();
//      }
//    }
//
//    //show_stats();
//    //plot_trajectories();
//  }
//
//  int r = 1;
//  for( auto &robot : robots){
//    query_time = query_time + robot->ocp_query_time.count();
//    std::cout << "R" << r++ <<  " Query Time: " << robot->ocp_query_time.count() << std::endl;
//  }
//  std::cout << "Query Time 1: " << query_time_1 << " iter: " << feasible_iter << std::endl;
//  std::cout << "Query Time: " << query_time << std::endl;
//  std::cout << "Cost: " << actual_cost() << std::endl;


}


//  if (robot->robot_id == 0) {
//    std::cout << *robot->data_shared->rho<< " " << std::endl;
//    std::cout << "R" << robot->robot_id << std::endl;
//    std::cout << "d " << std::endl;
//    std::cout << robot->J_model << std::endl;
//    std::cout << "d1 " << std::endl;
//    std::cout << robot->collisions_d->get_augmented_lagrangian() << std::endl;
//    std::cout << "d2 " << std::endl;
//    std::cout << robot->mu_col * robot->sum_g_col << std::endl;
//
//    std::cout << robot->data_shared->Xcurr(0) << std::endl;
//    std::cout << robot->data_shared->Xcurr(1) << std::endl;
//    std::cout << robot->data_shared->getSafetyDistance(0) << std::endl;
//    robot->data_shared->printMu12() ;
//    for (int r1 = 0; r1 < R; ++r1) {
//      for (int r2 = 0; r2 < R; ++r2) {
//        for (int k = 0; k < N + 1; ++k) {
//          if (r1 == r2)
//            continue;
//          std::cout << robot->data_shared->Mu12_val(r1, r2, k) << " ";
////      //std::cout << robot->data_shared->X(0, k) << " ";
//        }
//      }
//    }
//  }
//std::cout << std::endl;

//  robot->data_shared->printX();
//  MPI_Barrier(MPI_COMM_WORLD);
//  robot->data_shared->printY();
//  MPI_Barrier(MPI_COMM_WORLD);
//  std::cout << robot->data_shared->Xcurr(0) << std::endl;
//  MPI_Barrier(MPI_COMM_WORLD);
////  std::cout << robot->X_curr << std::endl;
//  MPI_Barrier(MPI_COMM_WORLD);
//  for (auto xcurr : X_mr_prev){
//    std::cout << "R" << std::endl;
//    std::cout << xcurr << std::endl;
//    std::cout << "R" << std::endl;
//    MPI_Barrier(MPI_COMM_WORLD);
//  }
