#include "CoupledProblem.hpp"

using namespace mropt::Problem;

void CoupledProblem::reset_ocp(){

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
        //s_opts["CPX_PARAM_BARALG"] = 3;
    }
    if (solver == "ipopt") {
        s_opts["print_level"] = 0;
        //s_opts["fixed_variable_treatment"] = "make_constraint";
        //s_opts["mu_strategy"] = "adaptive";
        //s_opts["mu_strategy"] = "adaptive";
        //s_opts["bound_relax_factor"] = 0;
        s_opts["linear_solver"] = "ma27";
    }

    // Coupled
    ocp = std::make_shared<Opti>();
    ocp->solver(solver, p_opts, s_opts);
    for (const auto &robot : robots) {
        robot->ocp_ = ocp;
    }

    //reset();
}

CoupledProblem &CoupledProblem::addRobot(const std::shared_ptr<Robot> &robot) {
    robots.push_back(robot);
    return *this;
}
CoupledProblem &CoupledProblem::addRobots(const std::vector<std::shared_ptr<Robot>> &robots_) {
    robots.insert(robots.end(),
                  robots_.begin(),
                  robots_.end());
    return *this;
}

void CoupledProblem::reset()
{
    //for(auto &robot : robots) robot->plot_ = true;
    reset_ocp();
    for (const auto &robot : robots)
        robot->resetQuery(*robot->ocp_);
}

void CoupledProblem::setup()
{
    for ( const auto &robot : robots )
        robot->setupQuery(*robot->ocp_);

    warm_start();
    // Coupled Setup
    MX f0{0.0};
    for ( const auto &robot : robots )
    {
        f0 = f0 + robot->J_model;
        robot->true_cost_ = [&](const DM& X, const DM& U){return robot->true_cost_l1penalty(X,U);};
    }
    // Collision Coupling constraints
    if (collisions_c)
        collisions_c->setup(*ocp, robots);
    if (collisions_c) {
        //std::cout << collisions_c->mu * collisions_c->sum_g << std::endl;
        ocp->minimize(f0 + collisions_c->mu * collisions_c->sum_g);
    } else {
        ocp->minimize(f0);
    }

    // Clear Plot vars
    if( plot_ || debug_ )
        for (const auto &robot : robots)
            robot->clear_plot_vars();
}
void CoupledProblem::convexify()
{
    for (const auto &robot : robots)
    {
        robot->convexify_dynamics(*robot->ocp_, robot->X_curr, robot->U_curr);
        //robot->dynamics.trustRegion().reset(ocp);
    }
    if(collisions_c) collisions_c->convexify(*ocp);// TODO: test
}
void CoupledProblem::warm_start() {
    for (const auto &robot : robots) {
        robot->ocp_->set_initial(robot->ss->X(), robot->X_curr);
        robot->ocp_->set_initial(robot->cs->U(), robot->U_curr);
    }
}
DM CoupledProblem::prev_actual_cost()
{
    DM total_cost{0.0};
    for (const auto &robot : robots)
    {
        DM t0f = DM({robot->p_.t0, robot->p_.tf});
        total_cost = total_cost + robot->cost->J(robot->X_curr, robot->U_curr, t0f);
    }
    return total_cost;
}
DM CoupledProblem::actual_cost()
{
    DM total_cost{0.0};
    for (const auto &robot : robots)
    {
        DM t0f = DM({robot->p_.t0, robot->p_.tf});
        total_cost = total_cost + robot->cost->J(robot->X_sol, robot->U_sol, t0f);
    }
    return total_cost;
}
/**
 * Compute cost associated with each robot, e.g. f0 + mu*(sum_g)
 * @param prev_true_cost
 * @param true_cost
 * @param model_cost
 * @param dynamics_violation
 * @param free_space_violation
 * @param pho
 * @param solution
 */
void CoupledProblem::compute_robot_costs_violations(
        std::vector<double> &pho,
        OptiSol & solution){
    for (int r_id = 0; r_id < robots.size(); ++r_id)
    {
        const auto &robot = robots[r_id];
        // Get/Set Solution
        robot->X_sol = solution.value(robot->ss->X());
        robot->U_sol = solution.value(robot->cs->U());
        //Compute Costs
        robot->dynamics_violation = robot->dynamics->J_max(robot->X_sol, robot->U_sol, DM({robot->p_.tf}));
        robot->free_space_violation = solution.value(robot->sum_g_free);
        robot->prev_true_cost = robot->true_cost_(robot->X_curr, robot->U_curr);
        robot->true_cost = robot->true_cost_(robot->X_sol, robot->U_sol);
        robot->model_cost = solution.value(robot->J_model);
        if (plot_ || debug_)
        { //Costs
            robot->true_cost_vec.push_back( robot->true_cost.scalar());
            robot->model_cost_vec.push_back(  robot->model_cost.scalar());
            robot->dynamics_violation_vec.push_back(robot->dynamics_violation.scalar());
            robot->free_space_violation_vec.push_back(robot->free_space_violation.scalar());
            //Print Costs (debug)
            std::cout << "Prev True Cost: " << robot->prev_true_cost << std::endl;
            std::cout << "True Cost: " << robot->true_cost  << std::endl;
            std::cout << "Model Cost: " << robot->model_cost  << std::endl;
            std::cout << "Dynamics Violation: " << robot->dynamics_violation << std::endl;
            std::cout << "Free Space Violation Computed: " << robot->free_space_violation << std::endl;
            //Path
            robot->x_sol = std::vector<double>(solution.value(robot->ss->x()));
            robot->y_sol = std::vector<double>(solution.value(robot->ss->y()));
            robot->o_sol = std::vector<double>(solution.value(robot->ss->o()));
            robot->x_sol_i.push_back(robot->x_sol);
            robot->y_sol_i.push_back(robot->y_sol);
            robot->o_sol_i.push_back(robot->o_sol);
            auto u_raw = std::vector<double>(solution.value(robot->cs->U()));
            robot->u_sol.clear();
            for(int u_id = 0; u_id < robot->cs->nu(); ++u_id){
                robot->u_sol.emplace_back(std::vector<double>());
                for(int k_u = 0; k_u < robot->cs->Nu(); ++k_u) {
                    robot->u_sol[u_id].push_back(u_raw[u_id + k_u * robot->cs->nu()]);
                }
            }
        }
        //Update total costs
        total_prev_true_cost = total_prev_true_cost + robot->prev_true_cost;
        total_true_cost = total_true_cost + robot->true_cost;
        total_model_cost = total_model_cost + robot->model_cost ;
        // Improvement Metric
        auto approx_reduction = robot->prev_true_cost  - robot->model_cost ;
        auto exact_reduction = robot->prev_true_cost  - robot->true_cost ;
        pho[r_id] = exact_reduction.scalar() / approx_reduction.scalar();
    }
}
/**
 * @brief Compute the associated cost for violations in coupled case,
 * e.g. constraints outside of robot class
 * @param cols_prev_true_cost
 * @param cols_true_cost
 * @param cols_model_cost
 * @param collisions_violation
 * @param solution
 */
void CoupledProblem::compute_collisions_costs_violations(
        OptiSol &solution)
{
    std::vector<DM> X_mr_curr{}; std::vector<DM> X_mr_sol{};
    for(const auto &robot: robots) {
        X_mr_curr.push_back(robot->X_curr);
        X_mr_sol.push_back(robot->X_sol);
    }
    collisions_c->prev_true_cost = collisions_c->mu_0 * collisions_c->J_real(X_mr_curr);
    collisions_c->true_cost = collisions_c->mu_0 * collisions_c->J_real(X_mr_sol);
    collisions_c->model_cost = collisions_c->mu_0 * solution.value(collisions_c->sum_g);
    collisions_c->violation = collisions_c->J_viol(X_mr_sol);
    //Update total costs
    total_prev_true_cost = total_prev_true_cost + collisions_c->prev_true_cost;
    total_true_cost = total_true_cost + collisions_c->true_cost;
    if(iter1){
        total_prev_true_cost = total_true_cost*2;
    }
    total_model_cost = total_model_cost + collisions_c->model_cost;
    if(plot_ || debug_) {
        std::cout << "Collisions Prev True Cost: " << collisions_c->prev_true_cost << std::endl;
        std::cout << "Collisions True Cost: " << collisions_c->true_cost << std::endl;
        std::cout << "Collisions Model Cost: " << collisions_c->model_cost << std::endl;
        std::cout << "Collisions Violation: " << collisions_c->violation << std::endl;
    }
}

void CoupledProblem::scp()
{
    std::vector<double> pho(robots.size());
    delta_x = DM(robots.size(), 1);
    //ocp->disp(std::cout, true);
    // Init
    prev_violation = -10;
    ocp_query_time = duration<double, std::milli>{0};
    //double solve_time = -MPI_Wtime();

    int iters = 0;
    while (true)
    { // Constraints Violation
        iter1 = true;
        for (const auto &robot : robots){ robot->dynamics->trustRegion().reset(*robot->ocp_);}
        while (true)
        { // Approximation Quality (convexity)
            convexify();
            while (true)
            { // Trust Region Quality
                auto t1 = high_resolution_clock::now();
                if(!solution){ solution = std::make_shared<OptiSol>(ocp->solve());}
                else {*solution = ocp->solve();}
                std::cout << "Iteration: " << iters << std::endl;
                iters++;
                auto t2 = high_resolution_clock::now();
                auto delta_t = t2 - t1;
                ocp_query_time = ocp_query_time + delta_t;
                // Warm_start next iter
                ocp->set_initial(ocp->x(), solution->value(ocp->x()));
                ocp->set_initial(ocp->lam_g(), solution->value(ocp->lam_g()));
                // Compute robot cost true and model, constraint violations and set solution
                total_prev_true_cost = 0.0; total_true_cost = 0.0; total_model_cost = 0.0;
                compute_robot_costs_violations( pho, *solution);
                if(collisions_c) compute_collisions_costs_violations(*solution);
                double pho_fleet = (total_prev_true_cost - total_true_cost).scalar() / (total_prev_true_cost - total_model_cost).scalar();
                delta_f = DM::abs(total_prev_true_cost - total_true_cost);

                //Plotting
                if (plot_ || debug_)
                {
                    std::cout << "Fleet Prev True Cost: " << total_prev_true_cost << std::endl;
                    std::cout << "Fleet True Cost: " << total_true_cost << std::endl;
                    std::cout << "Fleet Model Cost: " << total_model_cost << std::endl;
                    std::cout << "1) delta_f: " << delta_f.scalar() << std::endl;
                    std::cout << "1) delta_x: " << DM::mmax(delta_x).scalar() << std::endl;
                    for (auto &robot : robots)
                    {
                        robot->x_sol = std::vector<double>(solution->value(robot->ss->x()));
                        robot->y_sol = std::vector<double>(solution->value(robot->ss->y()));
                        robot->o_sol = std::vector<double>(solution->value(robot->ss->o()));
                        robot->x_sol_i.push_back(robot->x_sol);
                        robot->y_sol_i.push_back(robot->y_sol);
                        robot->o_sol_i.push_back(robot->o_sol);
                        auto u_raw = std::vector<double>(solution->value(robot->cs->U()));
                        robot->u_sol.clear();
                        for(int u_id = 0; u_id < robot->cs->nu(); ++u_id){
                            robot->u_sol.emplace_back(std::vector<double>());
                            for(int k_u = 0; k_u < robot->cs->Nu(); ++k_u) {
                                robot->u_sol[u_id].push_back(u_raw[u_id + k_u * robot->cs->nu()]);
                            }
                        }
                    }
                }
                //Check if model is good enough or not
                failed = false;
                bool break_ = handle_solution_coupled(pho, pho_fleet);
                //Break if trust region is too small
                for (auto &robot : robots) {
                    if (robot->dynamics->trustRegion().get_radius() < x_tol)
                        failed = true;
                }
                if (break_ || failed)
                    break;
                if (delta_f.scalar() <= f_tol || DM::mmax(delta_x).scalar() < x_tol)
                    break;

                iter1 = false;
            }

            //plot();
            if(iter1){
                iter1 = false;
                continue;
            }
            if(plot_ || debug_){
                std::cout << "delta_f: " << delta_f.scalar() << std::endl;
                std::cout << "delta_x: " << DM::mmax(delta_x).scalar() << std::endl;
            }
            if (failed)
                break; //trust radius region is minimum
            if (delta_f.scalar() <= f_tol || DM::mmax(delta_x).scalar() < x_tol)
                break;
        }


        bool constraint_violation = false;
        for (const auto &robot : robots)
        {
            if(plot_ || debug_){
                std::cout << robot->dynamics_violation << std::endl;
                std::cout << robot->free_space_violation << std::endl;
            }
            if(robot->dynamics_violation .scalar() > g_tol){
                robot->mu_d_0 = k * robot->mu_d_0;
                robot->ocp_->set_value(robot->mu_dynamics, robot->mu_d_0);
                constraint_violation = true;
            }
            if(robot->free_space_violation.scalar() > g_tol){//TODO: error free_space_violation
                robot->mu_f_0 = k * robot->mu_f_0;
                robot->ocp_->set_value(robot->mu_free, robot->mu_f_0);
                constraint_violation = true;
            }
        }
        if(collisions_c) {
            if(plot_ || debug_){
                std::cout << collisions_c->violation << std::endl;
            }
            if (collisions_c->violation.scalar() > collisions_c->col_.threshold) {
                collisions_c->mu_0 = k * collisions_c->mu_0;
                ocp->set_value(collisions_c->mu, collisions_c->mu_0);
                constraint_violation = true;
            }
        }
        double total_model_cost_nocol{0.0};
        for(const auto &robot : robots){
            total_model_cost_nocol = solution->value(robot->J_model_nocol).scalar();
        }
        if(total_model_cost.scalar() < 0 ||  total_model_cost_nocol < 0 ) break;
        if(!constraint_violation) break;
        if(!viol_improv(5)) break;

        //TODO : test true_cost funciton
        for ( const auto &robot : robots )
        {
            robot->true_cost_ = [&](const DM& X, const DM& U){return robot->true_cost_l1penalty(X,U);};
        }


    }

    //solve_time +=MPI_Wtime();
    double cost = actual_cost().scalar();
    std::cout << "Solve Time: " << ocp_query_time.count() << std::endl;
    std::cout << "Cost: " << actual_cost() << std::endl;
    std::cout << "Iterations: " << iters << std::endl;
    //std::cout << "Query Time: " << solve_time << std::endl;
    if(collisions_c->violation.scalar() >  collisions_c->col_.threshold + 10*g_tol) record_curr.fail_status = -1;
/*    for (const auto &robot : robots)
    {
        if(robot->dynamics_violation .scalar() > 10*g_tol ){ //  || robot->free_space_violation.scalar() > 10*g_tol
            record_curr.fail_status = -1;
        }
    }*/


    record_curr.R = robots.size();
    record_curr.t_1 = ocp_query_time.count()/1000;
    record_curr.t = ocp_query_time.count()/1000;
    record_curr.T = robots[0]->p_.tf;
    record_curr.N = robots[0]->p_.N;
    record_curr.L = robots[0]->shape->get_safety_radius();
    record_curr.cost = cost;
    record_curr.cost_1 = cost;
}



bool CoupledProblem::handle_solution_coupled(const std::vector<double> &pho, double pho_fleet){
    //  Trust Region Update
    for (int r_id = 0; r_id < robots.size(); r_id++){
        auto robot = robots[r_id];
        if(pho_fleet < pho1) {
            robot->dynamics->trustRegion().shrink(*robot->ocp_);
        }
        if(pho_fleet >= pho1) {
            robot->dynamics->trustRegion().expand(*robot->ocp_);
        }

        if(pho_fleet >= pho0) { //accept solution
            delta_x(r_id) =
                    (sum1(sum2(DM::abs(robot->X_sol - robot->X_curr))) + sum1(sum2(DM::abs(robot->U_sol - robot->U_curr))))
                    / (robots[r_id]->ss->Nx() + robots[r_id]->cs->Nu());
            robot->X_curr = robot->X_sol;
            robot->U_curr = robot->U_sol;
        }
    }

    if(pho_fleet < 0.0) { // Recovery mode
        DM actual_cost_before_bt = total_prev_true_cost;
        DM actual_cost_after_bt = 0.0;
        if(backtrack(actual_cost_after_bt)){
            delta_f = DM::abs(total_prev_true_cost - prev_actual_cost());
            if(plot_ || debug_){
                for(const auto &robot : robots) {
                    robot->true_cost_vec.pop_back();
                    robot->true_cost_vec.push_back(robot->true_cost_(robot->X_curr, robot->U_curr).scalar());
                }
            }
            failed = true;
            return true;
        }
    }

    // Overall fleet progress
    if (pho_fleet < pho0) //reject solution
    {
        //return false;
        return false;
    }
    else //accept solution
    {
        if (pho_fleet < pho1) //accept solution but solve again
        {
            return false;
        }
        //if (pho1 < pho && pho < pho2)// maintain
        if (pho_fleet >= pho1) //solution is good enough
        {
            return true;
        }
    }
    return true;

}

/**
 * Backtrack considering coupling collisions
 * @param true_cost
 * @return
 */
bool CoupledProblem::backtrack(DM &true_cost)
{
    std::vector<DM> X_before_bt, U_before_bt;
    for(const auto &robot : robots){
        X_before_bt.push_back(robot->X_curr);
        U_before_bt.push_back(robot->U_curr);
    }


    bool backtracked = false;
    double t = 1;
    double alpha = 0.1;
    double beta = 10; //2
    auto N = robots[0]->X_sol.size1();
    auto R = robots.size();
    std::vector<DM> dx = std::vector<DM>(R, DM::zeros(N,1));
    std::vector<DM> du = std::vector<DM>(R, DM::zeros(N,1));
    DM delta_true_cost{0.0}, model_cost{0.0};
    while(true){
        std::vector<DM> X_mr{}, X_mr_dx{};
        for (int r = 0; r < R; ++r){
            dx[r] = robots[r]->X_sol - robots[r]->X_curr;
            du[r] = robots[r]->U_sol - robots[r]->U_curr;
            X_mr.push_back(robots[r]->X_curr);
            X_mr_dx.push_back(robots[r]->X_curr + t*dx[r]);
        }
        delta_true_cost = 0.0; true_cost = 0.0; model_cost = 0.0;
        for (int r = 0; r < R; ++r){
            delta_true_cost = delta_true_cost + robots[r]->true_cost_(robots[r]->X_curr + t * dx[r],
                                                                      robots[r]->U_curr + t * du[r]);
            true_cost = true_cost + robots[r]->true_cost_(robots[r]->X_curr, robots[r]->U_curr);
            model_cost = model_cost + robots[r]->model_cost;
        }
        if(collisions_c) {
            delta_true_cost = delta_true_cost + collisions_c->mu_0 * collisions_c->J_real(X_mr_dx);
            true_cost = true_cost + collisions_c->mu_0 * collisions_c->J_real(X_mr);
            model_cost = model_cost + collisions_c->mu_0 * collisions_c->model_cost;
            collisions_c->violation = collisions_c->J_viol(X_mr);
        }
        if (delta_true_cost.scalar() <= true_cost.scalar()  - alpha * t * (true_cost.scalar() - model_cost.scalar()) )
            break;
        t = t / beta;
        for (int r = 0; r < R; ++r) {
            robots[r]->X_curr = robots[r]->X_curr + t * dx[r];
            robots[r]->U_curr = robots[r]->U_curr + t * du[r];
        }
        backtracked = true;
    }
    // Update delta_x accordingly
    for(int r_id = 0; r_id < R; ++r_id) {
        delta_x(r_id) =
                (sum1(sum2(DM::abs(X_before_bt[r_id] - robots[r_id]->X_curr)))
                 + sum1(sum2(DM::abs(U_before_bt[r_id] - robots[r_id]->U_curr))))
                / (robots[r_id]->ss->Nx() + robots[r_id]->cs->Nu());
    }
    for(const auto & robot :robots){
        robot->dynamics_violation = robot->dynamics->J_max(robot->X_curr, robot->U_curr, DM({robot->p_.tf}));
    }
    return backtracked;
}

/**
 * Checks if solution violation converged in percentage tol
 * @param tol
 * @return
 */
bool CoupledProblem::viol_improv(double tol){

    DM new_violation{0.0};
    for( const auto &robot : robots){
        new_violation = new_violation + robot->dynamics_violation;// + robot->free_space_violation;
    }
    if(plot_ || debug_){
        std::cout << new_violation << std::endl;
        std::cout << prev_violation << std::endl;
    }
    if( collisions_c ) new_violation = new_violation + collisions_c->violation;
    double violation_improvement = new_violation.scalar()/abs(prev_violation.scalar());
    prev_violation = new_violation;
    if(plot_ || debug_) std::cout << "Viol improv: " << violation_improvement << std::endl;
    if( violation_improvement >= 1-tol/100 && violation_improvement <= 1+tol/100 )
        return false;
    return true;
}

CoupledProblem& CoupledProblem::show_stats(){
    if( !plot_ ) std::cerr << "[CoupledProblem] Call allow_plotting() before solving" << std::endl;
    for(const auto &robot : robots){
        robot->plot();
    }
    return *this;
}

CoupledProblem& CoupledProblem::plot_trajectories() {
    if( !plot_ ) std::cerr << "[OPTPROBLEM] Call allow_plotting() before solving" << std::endl;
    if( plotter == nullptr ) {std::cerr << "[OPTPROBLEM] use set_plotter()" << std::endl; return *this;}
    if( plotter->odes.empty() ){//uninitialized
        std::vector<std::shared_ptr<mropt::Dynamics::ode>> odes;
        for(const auto &robot : robots){
            odes.push_back(robot->dynamics->ode_);
        }
        plotter->set_models(odes);
    }

    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> y;
    std::vector<std::vector<double>> o;
    std::vector<std::vector<std::vector<double>>> u;
    for(const auto &robot : robots){
        std::vector<double> x_r, y_r, o_r; std::vector<std::vector<double>> u_r;
        robot->get_solution(x_r,y_r,o_r, u_r);
        x.emplace_back(x_r); y.emplace_back(y_r); o.emplace_back(o_r); u.push_back(u_r);
    }
    plotter->plot_trajectory(x, y, o, u, robots[0]->p_.tf);
    return *this;
}

void CoupledProblem::solve(){
    auto t1 = high_resolution_clock::now();
    reset();
    setup();
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> delta_t = t2 - t1;

    scp();
    reset();

    record_curr.t_setup = (double)delta_t.count()/1000;
}


CoupledProblem& CoupledProblem::allow_plotting() {
    if( !plotter ) std::cerr << "[OPTPROBLEM] provide a plotter facility" << std::endl;
    for(auto &robot : this->robots){
        robot->plot_ = true;
    }
    plot_ = true;
    return *this;
}

CoupledProblem &CoupledProblem::debug_mode(){
    for(auto &robot : this->robots){
        robot->debug_ = true;
    }
    this->debug_ = true;
    return *this;
}
