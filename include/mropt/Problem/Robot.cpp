#include "Robot.hpp"
#include <matplot/matplot.h>

using namespace mropt::Problem;

Robot::Robot(const Robot::Params &P,
             const std::shared_ptr<mropt::ControlSpace::Control> &Cs,
             const std::shared_ptr<mropt::StateSpace::State> &Ss,
             const std::shared_ptr<mropt::cost::Cost> &Cost,
             const std::shared_ptr<mropt::freespace::FreeSpace> &Fspace,
             const std::shared_ptr<mropt::Dynamics::Transcription> &Dynamics,
             const std::shared_ptr<mropt::RobotShape::Footprint> &Shape)
        : p_(P), cs(Cs), ss(Ss), cost(Cost), fspace(Fspace), dynamics(Dynamics), shape(Shape) {
    setup();
}

/**
 * Generates a new robot by copying argument robot
 * @param robot
 */
Robot::Robot(const Robot &robot) {
    this->p_ = robot.p_;
    this->ss = robot.ss->clone();
    this->cs = robot.cs->clone();
    this->shape = robot.shape->clone();
    this->fspace = std::make_shared<mropt::freespace::FreeSpace>(*this->ss);
    this->cost = std::make_shared<mropt::cost::Cost>(*this->cs, *this->ss);
    const auto &ode_copy = robot.dynamics->ode_->clone(this->ss, this->cs, this->shape);
    const auto &ode_approx_copy = robot.dynamics->ode_approx_->clone(ode_copy);
    this->dynamics = robot.dynamics->clone(ode_approx_copy);

    setup();
}

DM Robot::true_cost_l1penalty(const DM &X, const DM &U) {
    DM t0f = DM({p_.t0, p_.tf});
    return cost->J(X, U, t0f)
           + mu_d_0 * dynamics->J_real(X, U, DM({p_.tf}))
           + mu_f_0 * fspace->J_real(X);
}

void Robot::plot() {
    //Solution and Plotting
    std::vector<double> o_sol_x = o_sol;
    std::vector<double> o_sol_y = o_sol;
    std::for_each(
            o_sol_x.begin(),
            o_sol_x.end(),
            [](double &o) {
                o = std::cos(o);
            });
    std::for_each(
            o_sol_y.begin(),
            o_sol_y.end(),
            [](double &o) {
                o = std::sin(o);
            });
    auto up = y_sol;
    std::for_each(
            up.begin(),
            up.end(),
            [](double &y) {
                y += 0.5;
            });
    auto down = y_sol;
    std::for_each(
            down.begin(),
            down.end(),
            [](double &y) {
                y -= 0.5;
            });
    plt::subplot(3, 2, 1);
    std::vector<double> x_s, y_s, o_s_x, o_s_y, up_s, down_s;
    std::vector<double> car_xl, car_yl, car_xr, car_yr;
    for (int i = 0; i < o_sol_x.size(); ++i) {
        //plt::clf();
        x_s.push_back(x_sol[i]);
        y_s.push_back(y_sol[i]);
        o_s_x.push_back(o_sol_x[i]);
        o_s_y.push_back(o_sol_y[i]);
        //up_s = (up[i]);
        //down_s = (down[i]);
        double x_w_l = x_sol[i] - 0.5 * sin(o_sol[i]);
        double y_w_l = y_sol[i] + 0.5 * cos(o_sol[i]);
        double x_w_r = x_sol[i] + 0.5 * sin(o_sol[i]);
        double y_w_r = y_sol[i] - 0.5 * cos(o_sol[i]);
        car_xl.push_back(x_w_l);
        car_xr.push_back(x_w_r);
        car_yl.push_back(y_w_l);
        car_yr.push_back(y_w_r);
    }
    plt::quiver(x_s, y_s, o_s_x, o_s_y);
    //plt::plot(x_sol,  y_sol);
    std::map<std::string, std::string> keywords;
    //keywords["alpha"] = "0.4";
    keywords["color"] = "grey";
    //keywords["hatch"] = "-";
    plt::scatter(car_xl, car_yl, 3, keywords);
    plt::scatter(car_xr, car_yr, 3, keywords);
    plt::xlim(0, 10);
    plt::ylim(0, 10);
    plt::axis("equal");
    plt::subplot(3, 2, 2);
    int Nt = o_sol_i.size();
    for (int it = 0; it < Nt; ++it) {
        x_sol = x_sol_i[it];
        y_sol = y_sol_i[it];
        std::map<std::string, std::string> keywords;
        //keywords["linewidth"] = std::to_string((double)it / Nt);
        keywords["linewidth"] = std::to_string((double) 2 * it / Nt);
        keywords["color"] = "grey";
        plt::plot(x_sol, y_sol, keywords);
        //plt::named_plot("test",x_sol,y_sol, "grey");
        //keywords["hatch"] = "-";
        //plt::scatter(std::vector{car_xl}, std::vector{car_yl}, 3, keywords);
        //plt::scatter(std::vector{car_xr}, std::vector{car_yr}, 3, keywords);
        //    plt::xlim(0, 10);
        //    plt::ylim(0, 10);
        //    plt::axis("equal");
    }
    plt::subplot(3, 2, 3);
    //int N_it = true_cost_vec.size();
    //auto iters = LinearSpacedVector(1, N_it, N_it);
    plt::title("Costs");
    plt::named_plot("True Cost", true_cost_vec);
    plt::named_plot("Model Cost", model_cost_vec);
    plt::legend();
    plt::subplot(3, 2, 4);
    plt::named_plot("Dynamics", dynamics_violation_vec, "y");
    plt::named_plot("Free Space", free_space_violation_vec, "r");
    plt::named_plot("Collisions", collisions_violation_vec, "b");
    plt::legend();
    plt::subplot(3, 2, 5);
    plt::title("Controls");
    for (int u_id = 0; u_id < u_sol.size(); u_id++) {
        plt::bar(u_sol[u_id]);
    }
    //plt::legend();

    plt::subplot(3, 2, 6);
    plt::title("States");
    plt::named_plot("x", x_sol);
    plt::named_plot("y", y_sol);
    plt::named_plot("o", o_sol);
    plt::legend();
    plt::show();

/*  matplot::figure();matplot::hold(true);
  matplot::plot(true_cost_vec, "r")->use_y2(true).line_width(3);
  matplot::hold(true);
  matplot::plot(model_cost_vec, "b")->use_y2(true).line_width(3);
    matplot::hold(true);
  matplot::plot(dynamics_violation_vec, "--y")->line_width(2);
    matplot::hold(true);
  matplot::plot(free_space_violation_vec, "--c")->line_width(2);
    matplot::hold(true);
  matplot::plot(collisions_violation_vec, "--g")->line_width(2);
  matplot::show();*/

}

bool Robot::backtrack() {
    DM t0f = DM({p_.t0, p_.tf});
    double t = 1;
    double alpha = 0.1;
    double beta = 2;

    bool backtracked = false;

    while (true) {
        DM dx = X_sol - X_curr;
        DM du = U_sol - U_curr;
        //DM delta_true_cost = cost.J(X_curr + t * dx, U_curr + t * du, t0f) + mu_d_0 * dynamics.J_real(X_curr + t * dx, U_curr + t * du, DM({p_.tf})) + mu_f_0 * fspace.J_real(X_curr + t * dx);
        DM delta_true_cost = true_cost_(X_curr + t * dx, U_curr + t * du);
        //DM curr_true_cost = cost.J(X_curr, U_curr, t0f) + mu_d_0 * dynamics.J_real(X_curr, U_curr, DM({p_.tf})) + mu_f_0 * fspace.J_real(X_curr);//TODO: swap this with true cost function
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
            = col_J_viol_(std::vector<casadi::DM>{{X_curr},
                                                  {solution->value(x2param)}})[0];
    return backtracked;
}

void Robot::resetQuery(Opti &ocp) {
    //Creating decision vars
    cs->setU(ocp.variable(cs->nu(), p_.N));
    ss->setX(ocp.variable(ss->nx(), p_.N + 1));
    //Current Trajectory Solution (iteration)
    traj0 = {
            std::make_shared<MX>(ocp.parameter(cs->nu(), cs->Nu())),
            std::make_shared<MX>(ocp.parameter(ss->nx(), ss->Nx()))};

    //Setup Transcription
    dynamics->setup(ocp, traj0.X0_, traj0.U0_, p_.t0, p_.tf, p_.N);

    // Bounds State
    ss->set_bounds(ocp);
    // Bounds Control
    cs->set_bounds(ocp);

    // Trust region
    const auto &trustregion_constraints_list = dynamics->get_trust_region_constraints();
    for (auto &trustregion_constraint: trustregion_constraints_list) {
        ocp.subject_to(trustregion_constraint <= 0);
    }

    // Constraints - Dynamics
    const auto &dynamics_constraints_list = dynamics->get_constraints();
    int g_t_num = dynamics_constraints_list.size();
    if (g_t_num != 0) {
        int dim_g_t = dynamics_constraints_list[0].size1(); //multiple shooting is equal to nx()
        MX slack_g_t_s = ocp.variable(dim_g_t, g_t_num);
        MX slack_g_t_t = ocp.variable(dim_g_t, g_t_num);

        for (int g_id = 0; g_id < g_t_num; ++g_id) {
            ocp.subject_to(slack_g_t_s(all, g_id) >= 0);
            ocp.subject_to(slack_g_t_t(all, g_id) >= 0);
            ocp.subject_to(dynamics_constraints_list[g_id] + slack_g_t_s(all, g_id) - slack_g_t_t(all, g_id) == 0);
        }
        MX sum_t = sum2(sum1(slack_g_t_t));
        MX sum_s = sum2(sum1(slack_g_t_s));
        sum_g_dynamics = sum_t + sum_s;
        max_g_dynamics = mmax(slack_g_t_t + slack_g_t_s);
    }
    // Cost
    mu_dynamics = ocp.parameter(1);
    mu_d_0 = mu_d_0_init;
    ocp.set_value(mu_dynamics, mu_d_0_init);
}

std::vector<double> LinearSpacedVector(double a, double b, std::size_t N) {
    double h = (b - a) / static_cast<double>(N - 1);
    std::vector<double> xs(N);
    std::vector<double>::iterator x;
    double val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
        *x = val;
    }
    return xs;
}

void Robot::initial_guess(SingleMission &mission) {
    std::vector<double> x_init, y_init, o_init;
    //Note: use mission.pas
    double x_0{mission.x0[0]}, y_0{mission.x0[1]}, o_0{mission.x0[2]};
    double x_f{mission.xf[0]}, y_f{mission.xf[1]}, o_f{mission.x0[2]};

    //No assignment
    if (mission.pas.empty()) {
        std::vector<double> x_k = LinearSpacedVector(x_0, x_f, p_.N + 1);
        x_init.insert(x_init.end(), x_k.begin(), x_k.end());
        std::vector<double> y_k = LinearSpacedVector(y_0, y_f, p_.N + 1);
        y_init.insert(y_init.end(), y_k.begin(), y_k.end());
        std::vector<double> o_k = LinearSpacedVector(o_0, o_f, p_.N + 1);
        o_init.insert(o_init.end(), o_k.begin(), o_k.end());
    }

    // TODO: uncomment block
    for (int pa_id = 0; pa_id < mission.pas.size(); ++pa_id) {//List of polygon assignments "pas"
        const auto &pa = mission.pas[pa_id];
        if (mission.pas.size() == 1) {//Stays in polygon
            std::vector<double> x_k = LinearSpacedVector(x_0, x_f, p_.N + 1);
            x_init.insert(x_init.end(), x_k.begin(), x_k.end());
            std::vector<double> y_k = LinearSpacedVector(y_0, y_f, p_.N + 1);
            y_init.insert(y_init.end(), y_k.begin(), y_k.end());
            std::vector<double> o_k = LinearSpacedVector(o_0, o_f, p_.N + 1);
            o_init.insert(o_init.end(), o_k.begin(), o_k.end());
            break;
        }
        //Changes Polygons
        if (pa_id == 0) {//First polygon change
            const auto &pa_next = mission.pas[pa_id + 1];
            auto &state = mropt::freespace::FreeSpace::poly_centers->at({pa.pid, pa_next.pid});
            std::vector<double> x_k = LinearSpacedVector(x_0, state.x, pa.kf - pa.k0 - 1);
            x_init.insert(x_init.end(), x_k.begin(), x_k.end());
            std::vector<double> y_k = LinearSpacedVector(y_0, state.y, pa.kf - pa.k0 - 1);
            y_init.insert(y_init.end(), y_k.begin(), y_k.end());
            std::vector<double> o_k = LinearSpacedVector(o_0, state.o, pa.kf - pa.k0 - 1);
            o_init.insert(o_init.end(), o_k.begin(), o_k.end());
        } else if (pa_id == mission.pas.size() - 1) {// Last polygon change
            const auto &pa_prev = mission.pas[pa_id - 1];
            auto &state = mropt::freespace::FreeSpace::poly_centers->at({pa_prev.pid, pa.pid});
            std::vector<double> x_k = LinearSpacedVector(state.x, x_f, pa.kf - pa.k0);
            x_init.insert(x_init.end(), x_k.begin(), x_k.end());
            std::vector<double> y_k = LinearSpacedVector(state.y, y_f, pa.kf - pa.k0);
            y_init.insert(y_init.end(), y_k.begin(), y_k.end());
            std::vector<double> o_k = LinearSpacedVector(state.o, o_f, pa.kf - pa.k0);
            o_init.insert(o_init.end(), o_k.begin(), o_k.end());
        } else { // In between
            const auto &pa_prev = mission.pas[pa_id - 1];
            const auto &state_prev = mropt::freespace::FreeSpace::poly_centers->at({pa_prev.pid, pa.pid});
            const auto &pa_next = mission.pas[pa_id + 1];
            const auto &state_next = mropt::freespace::FreeSpace::poly_centers->at({pa.pid, pa_next.pid});
            std::vector<double> x_k = LinearSpacedVector(state_prev.x, state_next.x, pa.kf - pa.k0);
            x_init.insert(x_init.end(), x_k.begin(), x_k.end());
            std::vector<double> y_k = LinearSpacedVector(state_prev.y, state_next.y, pa.kf - pa.k0);
            y_init.insert(y_init.end(), y_k.begin(), y_k.end());
            std::vector<double> o_k = LinearSpacedVector(state_prev.o, state_next.o, pa.kf - pa.k0);
            o_init.insert(o_init.end(), o_k.begin(), o_k.end());
        }
    }
    auto u_v_init = std::vector(p_.N, 0.0);
    auto u_w_init = std::vector(p_.N, 0.0);
/*  x_init =  std::vector(p_.N+1, 0.0); //TODO: comment me
  y_init =  std::vector(p_.N+1, 0.0);
  o_init =  std::vector(p_.N+1, 0.0);*/
    //Add Noise
//  for(auto &x : x_init){ x = x + UniformNoise(-0.1, 0.1);}
//  for(auto &y : y_init){ y = y + UniformNoise(-0.1, 0.1);}
//  for(auto &u_v : u_v_init){ u_v = u_v + UniformNoise(-0.01, 0.01);}
//  for(auto &u_w : u_w_init){ u_w = u_w + UniformNoise(-0.01, 0.01);}
    X_curr = DM({x_init, y_init, o_init});
    U_curr = DM({u_v_init, u_w_init});

    // TODO: each state_space and control_space should provide a way to compute this
    // This is an easy fix for now (so forever)
    if (X_curr.size1() != ss->nx_) {
        int n = ss->nx_ - X_curr.size1();
        while (n--) {
            auto x_fake = transpose(DM({std::vector(p_.N + 1, 0.0)}));
            std::cout << x_fake.size1() << std::endl;
            std::cout << x_fake.size2() << std::endl;
            X_curr = DM::vertcat({X_curr, x_fake});
        }
    }

    if (U_curr.size1() != cs->nu_) {
        int n = ss->nx_ - U_curr.size1();
        while (n--) {
            auto u_fake = transpose(DM({std::vector(p_.N + 1, 0.0)}));
            std::cout << u_fake.size1() << std::endl;
            std::cout << u_fake.size2() << std::endl;
            U_curr = DM::vertcat({U_curr, u_fake});
        }
    }
}

void Robot::setupQuery(Opti &ocp) {
    J_model = 0.0;
    //Mission x0 and xf - bvp
    if (missions.empty())
        std::cerr << "[OPTPROBLEM] no missions, add a mission" << std::endl;
    mission_curr = std::make_shared<SingleMission>(missions.back());
    missions.pop_back();// Delete mission from queue
    // Set initial guess
    initial_guess(*mission_curr);
//  // Mission Constraints
//  auto mission_constraints_list = mission_curr->get_constraints();
//  for (auto &mission_constraint : mission_constraints_list) {
//    ocp.subject_to(mission_constraint == 0);
//  }
    // Mission Constraints
    mission_curr->set_problem(J_model, ocp);

    // Free Space
    fspace->setup(ocp);
    const auto &fspace_constraints_list
            = fspace->get_constraints(mission_curr->pas);
    int g_f_num = fspace_constraints_list.size();
    if (g_f_num != 0) {
        int dim_g_f = fspace_constraints_list[0].size1();
        MX slack_g_f_t = ocp.variable(dim_g_f, g_f_num);
        for (int g_id = 0; g_id < g_f_num; ++g_id) {
            ocp.subject_to(slack_g_f_t(all, g_id) >= 0);
            ocp.subject_to(fspace_constraints_list[g_id] - slack_g_f_t(all, g_id) <= 0);
        }
        //Cost
        sum_g_free = sum2(sum1(slack_g_f_t));
        max_g_free = mmax(slack_g_f_t);
    }
    mu_free = ocp.parameter(1);
    mu_f_0 = mu_f_0_init;
    ocp.set_value(mu_free, mu_f_0_init);
    J_model = J_model + cost->integrated_cost(dynamics->t0, dynamics->tf, dynamics->N) + mu_dynamics * sum_g_dynamics
              + mu_free * sum_g_free;
    J_model_nocol = J_model; //TODO: remove me
}

void Robot::clear_plot_vars() {
    true_cost_vec.clear();
    model_cost_vec.clear();
    dynamics_violation_vec.clear();
    free_space_violation_vec.clear();
    x_sol.clear();
    y_sol.clear();
    o_sol.clear();
    u_sol.clear();
    xall_sol.clear();
    x_sol_i.clear();
    y_sol_i.clear();
    o_sol_i.clear();
}

void Robot::scp() {
    // Init
    prev_violation = -10;
    int iters = 0;
    while (true) { // Constraints Violation
        dynamics->trustRegion().reset(*ocp_);
        while (true) { // Approximation Quality (convexity)
            convexify_dynamics(*ocp_, X_curr, U_curr);
            convexify_decoupled_collisions();
            while (true) { // Trust Region Quality
                auto t1 = high_resolution_clock::now();
                if (!solution) {
                    solution = std::make_shared<OptiSol>(ocp_->solve());
                } else {
                    // Warm_start next iter
                    ocp_->set_initial(ocp_->x(), solution->value(ocp_->x()));
                    ocp_->set_initial(ocp_->lam_g(), solution->value(ocp_->lam_g()));
                    *solution = ocp_->solve(); //solve
                }
                std::cout << "Iteration: " << iters << std::endl;
                iters++;
                auto t2 = high_resolution_clock::now();
                auto delta_t = t2 - t1;
                ocp_query_time = ocp_query_time + delta_t;
                // Compute robot cost true and model, constraint violations and set solution
                prev_true_cost = 0.0;
                true_cost = 0.0;
                model_cost = 0.0;
                double pho = compute_robot_costs_violations();
                delta_f = DM::abs(prev_true_cost - true_cost) / true_cost;
                //Check if model is good enough or not
                failed = false;
                bool break_ = handle_solution(pho);
                //Break if trust region is too small
                if (dynamics->trustRegion().get_radius() < x_tol)
                    failed = true;
                if (break_ || failed)
                    break;
                if (delta_f.scalar() <= f_tol || DM::mmax(delta_x).scalar() < x_tol)
                    break;
            }

            //plot();
            std::cout << "delta_f: " << delta_f.scalar() << std::endl;
            std::cout << "delta_x: " << DM::mmax(delta_x).scalar() << std::endl;
            if (failed) break; //trust radius region is minimum || backtrack()
            if (std::abs(delta_f.scalar()) <= f_tol || DM::mmax(delta_x).scalar() < x_tol) {
                break;
            }
        }

        bool constraint_violation = false;
        std::cout << dynamics_violation << std::endl;
        std::cout << free_space_violation << std::endl;
        std::cout << collisions_violation << std::endl;
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
        if (collisions_violation.scalar() > g_tol) {
            mu_c_0 = k * mu_c_0;
            ocp_->set_value(mu_col, mu_c_0);
            constraint_violation = true;
        }

        if (!constraint_violation)
            break;
        if (!viol_improv(5))
            break;
    }

    std::cout << "Query Time: " << ocp_query_time.count() << std::endl;
    std::cout << "Iterations: " << iters << std::endl;
    //Cleanning //TODO : Check if this is necessary
    ocp_->set_value(mu_dynamics, mu_d_0_init);
    ocp_->set_value(mu_free, mu_f_0_init);
    mu_d_0 = mu_d_0_init;
    mu_f_0 = mu_f_0_init;
}


double Robot::compute_robot_costs_violations() {
    // Get/Set Solution
    X_sol = solution->value(ss->X());
    U_sol = solution->value(cs->U());
    //Compute Costs
    dynamics_violation = dynamics->J_max(X_sol, U_sol, DM({p_.tf}));
    free_space_violation = solution->value(sum_g_free);
    collisions_violation
            = col_J_viol_(std::vector<casadi::DM>{{X_curr},
                                                  {solution->value(x2param)}})[0];
    prev_true_cost = true_cost_(X_curr, U_curr);
    true_cost = true_cost_(X_sol, U_sol);
    model_cost = solution->value(J_model);
    if (plot_) {
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
        std::cout << "Model Cost No col: " << model_cost - model_cost_no_col << std::endl;
        std::cout << "True Cost No col: " << true_cost - true_cost_l1penalty(X_sol, U_sol) << std::endl;
//      std::cout << "Model Cost 2: " << solution->value(ocp_->f()) << std::endl;
        std::cout << "Dynamics Violation: " << dynamics_violation << std::endl;
//      std::cout << "Dynamics Violation2: " << dynamics->J_real(X_sol, U_sol, DM({p_.tf})) << std::endl;
//      std::cout << "Dynamics Violation3: " << solution->value(sum_g_dynamics) << std::endl;
        std::cout << "Free Space Violation Computed: " << free_space_violation << std::endl;
        std::cout << "Collisions Violation Computed: " << collisions_violation << std::endl;
        // For matplotlib plots
        x_sol = std::vector<double>(solution->value(ss->x()));
        y_sol = std::vector<double>(solution->value(ss->y()));
        o_sol = std::vector<double>(solution->value(ss->o()));
        x_sol_i.push_back(x_sol);
        y_sol_i.push_back(y_sol);
        o_sol_i.push_back(o_sol);
        // For trajectory plot
        // Gather states
        auto x_raw = std::vector<double>(solution->value(ss->X()));
        xall_sol.clear();
        for (int x_id = 0; x_id < ss->nx(); ++x_id) {
            xall_sol.emplace_back(std::vector<double>());
            for (int k_x = 0; k_x < ss->Nx(); ++k_x) {
                xall_sol[x_id].push_back(x_raw[x_id + k_x * ss->nx()]);
            }
        }
        // Gather controls
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

bool Robot::handle_solution(double pho) {
    //  Trust Region Update
    if (pho < pho1) {
        dynamics->trustRegion().shrink(*ocp_);
    }
    if (pho >= pho1) {
        dynamics->trustRegion().expand(*ocp_);
    }
    if (pho >= pho0) { //accept solution
        delta_x =
                (sum1(sum2(DM::abs(X_sol - X_curr))) + sum1(sum2(DM::abs(U_sol - U_curr))))
                / (ss->Nx() + cs->Nu());
        X_curr = X_sol;
        U_curr = U_sol;
    }
    // Check if recovery is needed
    if (pho < 0.0) { // Recovery mode
        DM t0f = DM({p_.t0, p_.tf});
        DM actual_cost_before_bt = cost->J(X_sol, U_sol, t0f);
        //Backtrack
        DM X_before_bt = X_curr;
        DM U_before_bt = U_curr;
        if (backtrack()) {
            //Note that X_curr is updated
            delta_x =
                    (sum1(sum2(DM::abs(X_before_bt - X_curr)))
                     + sum1(sum2(DM::abs(U_before_bt - U_curr))))
                    / (ss->Nx() + cs->Nu());
            DM actual_cost_after_bt = cost->J(X_curr, U_curr, t0f);
            //delta_f = delta_f - actual_cost_before_bt + actual_cost_after_bt; // Update overall delta_f (considers fleet)
            delta_f
                    = std::abs(
                    true_cost_(X_before_bt, U_before_bt).scalar()
                    - true_cost_(X_curr, U_curr).scalar()) / true_cost_(X_curr, U_curr).scalar(); //TODO: testing this
            //failed = true;

            return true; //break
        }
    }
    // Overall fleet progress
    if (pho < pho0) //reject solution
    {
        return false;
    } else //accept solution
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

bool Robot::viol_improv(double tol) {
    int no_improv = 0;
    //Dynamics
    DM new_dynamic_viol = dynamics_violation;
    if (debug_) {
        std::cout << new_dynamic_viol << std::endl;
        std::cout << prev_violation_dynamics << std::endl;
    }
    double dynamic_viol_improvement;
    if (prev_violation_dynamics.scalar() == 0) {
        dynamic_viol_improvement = 1;
    } else {
        dynamic_viol_improvement = new_dynamic_viol.scalar() / abs(prev_violation_dynamics.scalar());
    }
    prev_violation_dynamics = new_dynamic_viol;
    if (debug_) std::cout << "VIOL Dynamics: " << dynamic_viol_improvement << std::endl;
    if (dynamic_viol_improvement >= 1 - tol / 100 && dynamic_viol_improvement <= 1 + tol / 100)
        no_improv++;

    //Free Space
    /* DM new_freespace_viol = free_space_violation;
     if( debug_) {
         std::cout << new_freespace_viol << std::endl;
         std::cout << prev_violation_freespace << std::endl;
     }
     double freespace_viol_improvement = new_freespace_viol.scalar()/abs(prev_violation_freespace.scalar());
     prev_violation_freespace = new_freespace_viol;
     if( debug_ ) std::cout << "VIOL FreeSpace: " << freespace_viol_improvement << std::endl;
     if( freespace_viol_improvement >= 1-tol/100 && freespace_viol_improvement <= 1+tol/100 )
         no_improv++;*/

    //Collisions
    DM new_collisions_viol = collisions_violation;
    if (debug_) {
        std::cout << new_collisions_viol << std::endl;
        std::cout << prev_violation_collisions << std::endl;
    }
    double collisions_viol_improvement;
    if (prev_violation_collisions.scalar() == 0) {
        collisions_viol_improvement = 1;
    } else {
        collisions_viol_improvement = new_collisions_viol.scalar() / abs(prev_violation_collisions.scalar());
    }
    prev_violation_collisions = new_collisions_viol;
    if (debug_) std::cout << "VIOL Collisions: " << collisions_viol_improvement << std::endl;
    if (collisions_viol_improvement >= 1 - tol / 100 && collisions_viol_improvement <= 1 + tol / 100)
        no_improv++;

    if (no_improv == 2) return false; //nothing improved 1 for each type of viol: dynamics, col, freespace
    return true;
}

/*  bool Robot::viol_improv(double tol){
    DM new_violation = dynamics_violation + free_space_violation + collisions_violation;
    if( debug_) {
      std::cout << new_violation << std::endl;
      std::cout << prev_violation << std::endl;
    }
    double violation_improvement = new_violation.scalar()/abs(prev_violation.scalar());
    prev_violation = new_violation;
    if( debug_ ) std::cout << "Viol improv: " << violation_improvement << std::endl;
    if( violation_improvement >= 1-tol/100 && violation_improvement <= 1+tol/100 )
      return false;
    return true;
  }*/

double Robot::UniformNoise(double start, double end) {
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double>
            distribution(start, end);
    return distribution(generator);
}


