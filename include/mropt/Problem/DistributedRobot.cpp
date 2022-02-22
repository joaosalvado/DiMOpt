//
// Created by ohmy on 2021-09-16.
//

#include "DistributedRobot.h"

using namespace mropt::Problem;

DistributedRobot::DistributedRobot(
        int robot_id_,
        const Robot::Params &P,
        const std::shared_ptr<mropt::ControlSpace::Control> &Cs,
        const std::shared_ptr<mropt::StateSpace::State> &Ss,
        const std::shared_ptr<mropt::cost::Cost> &Cost,
        const std::shared_ptr<mropt::freespace::FreeSpace> &Fspace,
        const std::shared_ptr<mropt::Dynamics::Transcription> &Dynamics,
        const std::shared_ptr<mropt::RobotShape::Footprint> &Shape,
        const std::shared_ptr<mropt::collisions::DistributedCollisions> &Collisions)
        : Robot(P, Cs, Ss, Cost, Fspace, Dynamics, Shape),
          collisions_d(Collisions),
          robot_id(robot_id_){}


void DistributedRobot::scp_d() {
//  if(robot_id==1) {
//    std::cout << "robot: " << robot_id << std::endl;
//    ocp_->disp(std::cout, true);
//  }
    // Init
    prev_violation = -10;
    prev_violation_dynamics = -10;
    prev_violation_freespace = -10;
    prev_violation_collisions = -10;
    this->clear_plot_vars(); //TODO: remove me
    int iters = 0;
    while (true) { // Constraints Violation
        iter1 = true;//TODO: debugging true before
        dynamics->trustRegion().reset(*ocp_);
        while (true) { // Approximation Quality (convexity)
            convexify_dynamics(*ocp_, X_curr, U_curr);
            collisions_d->convexify(); //remove me
            while (true) { // Trust Region Quality
                auto t1 = high_resolution_clock::now();
                if (!solution || prev_solved_dummy_ocp) {
                    prev_solved_dummy_ocp = false;
                    solution = std::make_shared<OptiSol>(ocp_->solve());
                    //x2param_list = solution->value(x2param);
                } else {
                    // Warm_start next iter
                    ocp_->set_initial(ocp_->x(), solution->value(ocp_->x()));
                    //ocp_->set_initial(ocp_->lam_g(), solution->value(ocp_->lam_g()));
                    *solution = ocp_->solve(); //solve
                }
                if(plot_ || debug_)std::cout << "Iteration: " << iters << std::endl;
                iters++;
                auto t2 = high_resolution_clock::now();
                auto delta_t = t2 - t1;
                ocp_query_time = ocp_query_time + delta_t;
                // Compute robot cost true and model, constraint violations and set solution
                prev_true_cost = 0.0;
                true_cost = 0.0;
                model_cost = 0.0;
                double pho = compute_robot_costs_violations_d();
                delta_f =  DM::abs(prev_true_cost - true_cost)/ true_cost;
                //Check if model is good enough or not
                failed = false;
                bool break_ = handle_solution_d(pho);
                //Break if trust region is too small
                if (dynamics->trustRegion().get_radius() < x_tol)
                    failed = true;
                if (break_ || failed)
                    break;
                if (delta_f.scalar() <= f_tol || DM::mmax(delta_x).scalar() < x_tol)
                    break;
                //if (DM::mmax(delta_x).scalar() < x_tol) break;

                iter1 = false;
            }
            if(iter1){
                iter1 = false;
                continue;
            }
            if(plot_ || debug_) {
                std::cout << "delta_f: " << delta_f.scalar() << std::endl;
                std::cout << "delta_x: " << DM::mmax(delta_x).scalar() << std::endl;
            }
            if (failed) break; //trust radius region is minimum || backtrack()
            if (delta_f.scalar() <= f_tol || DM::mmax(delta_x).scalar() < x_tol)
                break;
            /*if ( DM::mmax(delta_x).scalar() < x_tol){
                break;
            }*/
        }

        bool constraint_violation = false;
        if(plot_ || debug_){
            std::cout << dynamics_violation << std::endl;
            std::cout << free_space_violation << std::endl;
            std::cout << collisions_violation << std::endl;
        }

        if (dynamics_violation.scalar() > g_tol) {
            mu_d_0 = k * mu_d_0;//TODO: x100 for dynamics (testing)
            ocp_->set_value(mu_dynamics, mu_d_0);
            constraint_violation = true;
        }
        if (free_space_violation.scalar() > g_tol) {
            mu_f_0 = k * mu_f_0;
            ocp_->set_value(mu_free, mu_f_0);
            constraint_violation = true;
        }
        if (std::sqrt(collisions_violation.scalar()) > this->collisions_d->col_.threshold ) {
            mu_c_0 = k * mu_c_0;
            ocp_->set_value(mu_col, mu_c_0);
            constraint_violation = true;
        }
        //If both dynamics and coll then 10x dynamics
        if (dynamics_violation.scalar() > g_tol &&
            std::sqrt(collisions_violation.scalar()) > this->collisions_d->col_.threshold ) {
            mu_d_0 = k * mu_d_0;//TODO: x100 for dynamics (testing)
            ocp_->set_value(mu_dynamics, mu_d_0);
            constraint_violation = true;
        }

        if (!constraint_violation)
            break;
        if (!viol_improv(5))
            break;
        if(model_cost.scalar() < 0 ||  solution->value(J_model_nocol).scalar() < 0 ) break; //TODO : test this

        true_cost_ = [&](const DM &X, const DM &U) {
            DM result{0.0};
            result += true_cost_l1penalty(X, U);
            result += collisions_d->J_al_->operator()
                    (std::vector<casadi::DM>{{X}, {*data_shared->rho}})[0];
            //result +=     robot->mu_c_0 * robot->solution->value(robot->col_J_real_);
            result += mu_c_0 *
                      col_J_real_fast_( std::vector<casadi::DM>{{X}, {x2param_list}})[0];
            return result;
        };
    }

    if(debug_) {
        std::cout << "Query Time: " << ocp_query_time.count() << std::endl;
        std::cout << "Iterations: " << iters << std::endl;
        //std::cout << "Backtracks: " <<  backtrack_counter << std::endl;
    }
    //Cleanning //TODO : Check if this is necessary
    ocp_->set_value(mu_dynamics, mu_d_0_init);
    ocp_->set_value(mu_free, mu_f_0_init);
    ocp_->set_value(mu_col, mu_c_0_init);
    mu_d_0 = mu_d_0_init;
    mu_f_0 = mu_f_0_init;
    mu_c_0 = mu_c_0_init;
    // Store the controls that are not in shared data unless we allow plotting
    auto u_raw = std::vector<double>(solution->value(cs->U()));
    u_sol.clear();
    for (int u_id = 0; u_id < cs->nu(); ++u_id) {
        u_sol.emplace_back(std::vector<double>());
        for (int k_u = 0; k_u < cs->Nu(); ++k_u) {
            u_sol[u_id].push_back(u_raw[u_id + k_u * cs->nu()]);
        }
    }

}


bool DistributedRobot::backtrack_d() {
    DM t0f = DM({p_.t0, p_.tf});
    double t = 1;
    double alpha = 0.1;
    double beta = 10;
    if(debug_) backtrack_counter = 0;
    bool backtracked = false;
    while (true) {
        if(debug_) backtrack_counter++;
        DM dx = X_sol - X_curr;
        DM du = U_sol - U_curr;
        DM delta_true_cost = true_cost_(DM::plus(X_curr, DM::mtimes(t, dx)), DM::plus(U_curr, DM::mtimes(t, du)));
        DM curr_true_cost = true_cost_(X_curr, U_curr);
        if (delta_true_cost.scalar()
            <= curr_true_cost.scalar() - alpha * t * (curr_true_cost.scalar() - model_cost.scalar()))
            break;
        t = t / beta;
        X_curr = X_curr + t * dx;
        U_curr = U_curr + t * du;
        backtracked = true;
    }
    // Update violations
    dynamics_violation = dynamics->J_max(X_curr, U_curr, DM({p_.tf}));
    collisions_violation
            = col_J_viol_( std::vector<casadi::DM>{{X_curr}, {x2param_list}})[0];
    return backtracked;
}


double DistributedRobot::compute_robot_costs_violations_d() {
    // Get/Set Solution
    X_sol = solution->value(ss->X());
    U_sol = solution->value(cs->U());
    //Compute Costs
    dynamics_violation = dynamics->J_max(X_sol, U_sol, DM({p_.tf}));
    free_space_violation = solution->value(sum_g_free);
    /*collisions_violation = solution->value(sum_g_col);*/
    collisions_violation
            = col_J_viol_( std::vector<casadi::DM>{{X_sol}, {x2param_list}})[0];//TODO: test this due to set_J_voil_per_robot
    true_cost = true_cost_(X_sol, U_sol);
    if(!iter1){
        prev_true_cost = true_cost_(X_curr, U_curr);
    } else{
        prev_true_cost = true_cost*2;
    }

//  std::cout << solution->opti().return_status() << std::endl;
//auto a = solution->opti().stats();
//  std::cout << solution->opti().stats()<< std::endl;
    model_cost = solution->value(J_model);
    if (plot_ || debug_) {
        true_cost_vec.push_back(true_cost.scalar());
        model_cost_vec.push_back(model_cost.scalar());
        dynamics_violation_vec.push_back(dynamics_violation.scalar());
        free_space_violation_vec.push_back(free_space_violation.scalar());
        collisions_violation_vec.push_back(collisions_violation.scalar());
        //Print Costs (debug)
        auto model_cost_no_col = solution->value(J_model_nocol);
        std::cout << "Prev True Cost: " << prev_true_cost << std::endl;
        std::cout << "True Cost: " << true_cost << std::endl;
        std::cout << "Model Cost: " << model_cost << std::endl;
        std::cout << "True Cost l1: " << this->true_cost_l1penalty(X_sol, U_sol)  << std::endl;
        std::cout << "True cost Consensus: " <<  collisions_d->J_al_->operator()
                (std::vector<casadi::DM>{{X_sol}, {*data_shared->rho}})[0] << std::endl;
        std::cout << "True cost collisions: " << mu_c_0*col_J_real_fast_( std::vector<casadi::DM>{{X_sol}, {x2param_list}})[0] << std::endl;
        std::cout << "Model l1: " << solution->value(J_model_nocol) << std::endl;
        std::cout << "2) Model l1 collisions: " <<  solution->value(mu_col*sum_g_col) << std::endl;
        /* std::cout << "Model Cost col: " << model_cost - model_cost_no_col << std::endl;
         std::cout << "True Cost No col: " << true_cost - true_cost_l1penalty(X_sol,U_sol) << std::endl;*/
//      std::cout << "Model Cost 2: " << solution->value(ocp_->f()) << std::endl;
        std::cout << "Dynamics Violation: " << dynamics_violation << std::endl;
        std::cout << "Dynamics Violation2: " << dynamics->J_real(X_sol, U_sol, DM({p_.tf})) << std::endl;
        std::cout << "Dynamics Violation3: " << solution->value(sum_g_dynamics) << std::endl;
        std::cout << "Free Space Violation Computed: " << free_space_violation << std::endl;
        std::cout << "Collisions Violation : " << collisions_violation << std::endl;
        std::cout << "Collisions Violation Model: " << solution->value(sum_g_col) << std::endl;
        std::cout << "Collisions Violation Jreal: " << solution->value(col_J_real_) << std::endl;
        x_sol = std::vector<double>(solution->value(ss->x()));
        y_sol = std::vector<double>(solution->value(ss->y()));
        o_sol = std::vector<double>(solution->value(ss->o()));
        x_sol_i.push_back(x_sol);
        y_sol_i.push_back(y_sol);
        o_sol_i.push_back(o_sol);
        auto u_raw = std::vector<double>(solution->value(cs->U()));
        u_sol.clear();
        for (int u_id = 0; u_id < cs->nu(); ++u_id) {
            u_sol.emplace_back(std::vector<double>());
            for (int k_u = 0; k_u < cs->Nu(); ++k_u) {
                u_sol[u_id].push_back(u_raw[u_id + k_u * cs->nu()]);
            }
        }
    }
    // Improvement Metric
    auto approx_reduction = prev_true_cost - model_cost;
    auto exact_reduction = prev_true_cost - true_cost;
    return exact_reduction.scalar() / approx_reduction.scalar();
}

bool DistributedRobot::handle_solution_d(double pho)
{
    //  Trust Region Update
    if(pho < pho1) {
        dynamics->trustRegion().shrink(*ocp_);
    }
    if(pho >= pho1) {
        dynamics->trustRegion().expand(*ocp_);
    }
    if(pho >= pho0) { //accept solution
        delta_x =
                (sum1(sum2(DM::abs(X_sol - X_curr))) + sum1(sum2(DM::abs(U_sol - U_curr))))
                / (ss->Nx() + cs->Nu());
        X_curr = X_sol;
        U_curr = U_sol;
    }
    // Check if recovery is needed
    if(pho < 0.0) { // Recovery mode
        this->failed = true;
        DM t0f = DM({p_.t0, p_.tf});
        DM actual_cost_before_bt = cost->J(X_sol, U_sol, t0f);
        //Backtrack
        DM X_before_bt = X_curr;
        DM U_before_bt = U_curr;
        if ( backtrack_d() ) {
            //Note that X_curr is updated
            delta_x =
                    (sum1(sum2(DM::abs(X_before_bt - X_curr)))
                     + sum1(sum2(DM::abs(U_before_bt - U_curr))))
                    / (ss->Nx() + cs->Nu());
            DM actual_cost_after_bt = cost->J(X_curr, U_curr, t0f);
            //delta_f = delta_f - actual_cost_before_bt + actual_cost_after_bt; // Update overall delta_f (considers fleet)
            delta_f
                    =  std::abs(
                    true_cost_(X_before_bt, U_before_bt).scalar()
                    - true_cost_(X_curr, U_curr).scalar() ) / true_cost_(X_curr, U_curr).scalar() ; //TODO: testing this

            return true; //break
        }
    }
    // Overall fleet progress
    if (pho < pho0) //reject solution
    {
        return false;
    }
    else //accept solution
    {
        if (pho < pho1) //accept solution but solve again
        {
            return false;
        }
        //if (pho1 < pho && pho < pho2)// maintain
        if (pho >= pho1) //solution is good enough
        {
            return true;
        }
    }
    return true;
}


double* DistributedRobot::get_x_curr(){
    //std::cout << this->X_curr((int)State::POS::x, all) << std::endl;
    auto x = this->X_curr((int)mropt::StateSpace::State::POS::x, all).ptr();
    return  x;
}
double* DistributedRobot::get_y_curr(){
    auto y = this->X_curr((int)mropt::StateSpace::State::POS::y, all);

    return y->data();
}



/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// REPEATED CODE
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////



void DistributedRobot::scp_d_nocol() {
//  if(robot_id==1) {
//    std::cout << "robot: " << robot_id << std::endl;
//    ocp_->disp(std::cout, true);
//  }
    // Init
    prev_solved_dummy_ocp = true;
    prev_violation = -10;
    prev_violation_dynamics = -10;
    prev_violation_freespace = -10;
    int iters = 0;
    while (true) { // Constraints Violation
        iter1 = true;
        dynamics->trustRegion().reset(*ocp_);
        while (true) { // Approximation Quality (convexity)
            convexify_dynamics(*ocp_, X_curr, U_curr);
            //collisions_d->convexify(); //remove me
            while (true) { // Trust Region Quality
                auto t1 = high_resolution_clock::now();
                if (!solution) {
                    solution = std::make_shared<OptiSol>(ocp_->solve());
                    //x2param_list = solution->value(x2param);
                } else {
                    // Warm_start next iter
                    ocp_->set_initial(ocp_->x(), solution->value(ocp_->x()));
                    //ocp_->set_initial(ocp_->lam_g(), solution->value(ocp_->lam_g()));
                    *solution = ocp_->solve(); //solve
                }
                if(plot_ || debug_)std::cout << "Iteration: " << iters << std::endl;
                iters++;
                auto t2 = high_resolution_clock::now();
                auto delta_t = t2 - t1;
                ocp_query_time = ocp_query_time + delta_t;
                // Compute robot cost true and model, constraint violations and set solution
                prev_true_cost = 0.0;
                true_cost = 0.0;
                model_cost = 0.0;
                double pho = compute_robot_costs_violations_d_nocol();
                delta_f =  DM::abs(prev_true_cost - true_cost)/ true_cost;
                //Check if model is good enough or not
                failed = false;
                bool break_ = handle_solution_d_nocol(pho);
                //Break if trust region is too small
                if (dynamics->trustRegion().get_radius() < x_tol)
                    failed = true;
                if (break_ || failed)
                    break;
                if (delta_f.scalar() <= f_tol || DM::mmax(delta_x).scalar() < x_tol)
                    break;
                iter1 = false;
            }
            if(iter1){
                iter1 = false;
                continue;
            }
            //plot();
            if(plot_ || debug_) {
                std::cout << "delta_f: " << delta_f.scalar() << std::endl;
                std::cout << "delta_x: " << DM::mmax(delta_x).scalar() << std::endl;
            }
            if (failed) break; //trust radius region is minimum || backtrack()
            if (std::abs(delta_f.scalar()) <= f_tol || DM::mmax(delta_x).scalar() < x_tol){
                break;
            }
        }

        bool constraint_violation = false;

        if(plot_ || debug_){
            std::cout << dynamics_violation << std::endl;
            std::cout << free_space_violation << std::endl;
        }

        if (dynamics_violation.scalar() > g_tol) {
            mu_d_0 = k * mu_d_0;
            ocp_->set_value(mu_dynamics, mu_d_0);
            constraint_violation = true;
        }
        if (free_space_violation.scalar() > g_tol) {
            mu_f_0 = k * mu_f_0;
            ocp_->set_value(mu_free, mu_f_0);
            constraint_violation = true;
        }
        if (!constraint_violation)
            break;
        if (!viol_improv(5))
            break;
        if(model_cost.scalar() < 0 ||  solution->value(J_model_nocol).scalar() < 0 ) break; //TODO : test this
    }

    if(debug_) {
        std::cout << "Query Time: " << ocp_query_time.count() << std::endl;
        std::cout << "Iterations: " << iters << std::endl;
        std::cout << "Backtracks: " << backtrack_counter << std::endl;
    }
    //Cleanning //TODO : Check if this is necessary
    ocp_->set_value(mu_dynamics, mu_d_0_init);
    ocp_->set_value(mu_free, mu_f_0_init);
    mu_d_0 = mu_d_0_init;
    mu_f_0 = mu_f_0_init;

    //Get u for solution trajectory
    auto u_raw = std::vector<double>(solution->value(cs->U()));
    u_sol.clear();
    for (int u_id = 0; u_id < cs->nu(); ++u_id) {
        u_sol.emplace_back(std::vector<double>());
        for (int k_u = 0; k_u < cs->Nu(); ++k_u) {
            u_sol[u_id].push_back(u_raw[u_id + k_u * cs->nu()]);
        }
    }
}





bool DistributedRobot::backtrack_d_nocol() {
    DM t0f = DM({p_.t0, p_.tf});
    double t = 1;
    double alpha = 0.1;
    double beta = 10;
    if(debug_) backtrack_counter = 0;
    bool backtracked = false;
    while (true) {
        if(debug_) backtrack_counter++;
        DM dx = X_sol - X_curr;
        DM du = U_sol - U_curr;
        DM delta_true_cost = true_cost_(DM::plus(X_curr, DM::mtimes(t, dx)), DM::plus(U_curr, DM::mtimes(t, du)));
        DM curr_true_cost = true_cost_(X_curr, U_curr);
        if (delta_true_cost.scalar()
            <= curr_true_cost.scalar() - alpha * t * (curr_true_cost.scalar() - model_cost.scalar()))
            break;
        t = t / beta;
        X_curr = X_curr + t * dx;
        U_curr = U_curr + t * du;
        backtracked = true;
    }
    // Update violations
    dynamics_violation = dynamics->J_max(X_curr, U_curr, DM({p_.tf}));
    return backtracked;
}


double DistributedRobot::compute_robot_costs_violations_d_nocol() {
    // Get/Set Solution
    X_sol = solution->value(ss->X());
    U_sol = solution->value(cs->U());
    //Compute Costs
    dynamics_violation = dynamics->J_max(X_sol, U_sol, DM({p_.tf}));
    free_space_violation = solution->value(sum_g_free);
    true_cost = true_cost_(X_sol, U_sol);
    if(!iter1){
        prev_true_cost = true_cost_(X_curr, U_curr);
    } else{
        prev_true_cost = 2*true_cost;
    }
    model_cost = solution->value(J_model);
    if (plot_ || debug_) {
        true_cost_vec.push_back(true_cost.scalar());
        model_cost_vec.push_back(model_cost.scalar());
        dynamics_violation_vec.push_back(dynamics_violation.scalar());
        free_space_violation_vec.push_back(free_space_violation.scalar());
        //Print Costs (debug)
        auto model_cost_no_col = solution->value(J_model_nocol);
        std::cout << "Prev True Cost: " << prev_true_cost << std::endl;
        std::cout << "True Cost: " << true_cost << std::endl;
        std::cout << "True Cost l1: " << this->true_cost_l1penalty(X_sol, U_sol)  << std::endl;
        std::cout << "Model Cost: " << model_cost << std::endl;
        std::cout << "Model l1: " << solution->value(J_model_nocol) << std::endl;
        std::cout << "Dynamics Violation: " << dynamics_violation << std::endl;
        std::cout << "Dynamics Violation2: " << dynamics->J_real(X_sol, U_sol, DM({p_.tf})) << std::endl;
        std::cout << "Dynamics Violation3: " << solution->value(sum_g_dynamics) << std::endl;
        std::cout << "Free Space Violation Computed: " << free_space_violation << std::endl;

        x_sol = std::vector<double>(solution->value(ss->x()));
        y_sol = std::vector<double>(solution->value(ss->y()));
        o_sol = std::vector<double>(solution->value(ss->o()));
        x_sol_i.push_back(x_sol);
        y_sol_i.push_back(y_sol);
        o_sol_i.push_back(o_sol);
        auto u_raw = std::vector<double>(solution->value(cs->U()));
        u_sol.clear();
        for (int u_id = 0; u_id < cs->nu(); ++u_id) {
            u_sol.emplace_back(std::vector<double>());
            for (int k_u = 0; k_u < cs->Nu(); ++k_u) {
                u_sol[u_id].push_back(u_raw[u_id + k_u * cs->nu()]);
            }
        }
    }
    // Improvement Metric
    auto approx_reduction = prev_true_cost - model_cost;
    auto exact_reduction = prev_true_cost - true_cost;
    return exact_reduction.scalar() / approx_reduction.scalar();
}

bool DistributedRobot::handle_solution_d_nocol(double pho)
{
    //  Trust Region Update
    if(pho < pho1) {
        dynamics->trustRegion().shrink(*ocp_);
    }
    if(pho >= pho1) {
        dynamics->trustRegion().expand(*ocp_);
    }
    if(pho >= pho0) { //accept solution
        delta_x =
                (sum1(sum2(DM::abs(X_sol - X_curr))) + sum1(sum2(DM::abs(U_sol - U_curr))))
                / (ss->Nx() + cs->Nu());
        X_curr = X_sol;
        U_curr = U_sol;
    }
    // Check if recovery is needed
    if(pho < 0.0) { // Recovery mode
        DM t0f = DM({p_.t0, p_.tf});
        DM actual_cost_before_bt = cost->J(X_sol, U_sol, t0f);
        //Backtrack
        DM X_before_bt = X_curr;
        DM U_before_bt = U_curr;
        if ( backtrack_d_nocol() ) {
            //Note that X_curr is updated
            delta_x =
                    (sum1(sum2(DM::abs(X_before_bt - X_curr)))
                     + sum1(sum2(DM::abs(U_before_bt - U_curr))))
                    / (ss->Nx() + cs->Nu());
            DM actual_cost_after_bt = cost->J(X_curr, U_curr, t0f);
            //delta_f = delta_f - actual_cost_before_bt + actual_cost_after_bt; // Update overall delta_f (considers fleet)
            delta_f
                    =  std::abs(
                    true_cost_(X_before_bt, U_before_bt).scalar()
                    - true_cost_(X_curr, U_curr).scalar() ) / true_cost_(X_curr, U_curr).scalar() ; //TODO: testing this
            this->failed = true;

            return true; //break
        }
    }
    // Overall fleet progress
    if (pho < pho0) //reject solution
    {
        return false;
    }
    else //accept solution
    {
        if (pho < pho1) //accept solution but solve again
        {
            return false;
        }
        //if (pho1 < pho && pho < pho2)// maintain
        if (pho >= pho1) //solution is good enough
        {
            return true;
        }
    }
    return true;
}