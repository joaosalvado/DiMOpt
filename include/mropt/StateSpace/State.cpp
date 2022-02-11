#include "State.hpp"

using namespace mropt::StateSpace;

void State::set_lower_bounds(std::vector<double> lb)
{
    if (lb.size() < nx())
    {
        std::cerr << "[State] lower bounds have the less size than state\n";
    }
    if (lb.size() > nx())
    {
        std::cerr << "[State] lower bounds have the bigger size than state\n";
        return;
    }
    lb_ = lb;
}

void State::set_upper_bounds(std::vector<double> ub)
{
    if (ub.size() < nx())
    {
        std::cerr << "[State] upper bounds have the less size than state\n";
    }
    if (ub.size() > nx())
    {
        std::cerr << "[State] upper bounds have the bigger size than state\n";
        return;
    }
    ub_ = ub;
}

std::list<casadi::MX> State::get_constraints()
{
    std::list<casadi::MX> constraints;
    for (int x_id = 0; x_id < lb_.size(); ++x_id)
    {
        constraints.push_back(-X_(x_id, all) + lb_[x_id]);
    }
    for (int x_id = 0; x_id < ub_.size(); ++x_id)
    {
        constraints.push_back(X_(x_id, all) - ub_[x_id]);
    }
    return constraints;
}

void State::set_bounds(casadi::Opti &ocp)
{
  if(lb_.empty() || ub_.empty()) return;
    for (int x_id = 0; x_id < lb_.size(); ++x_id)
    {
        //ocp.bounded(lb_[x_id] , X_(x_id, all), ub_[x_id]);
        ocp.subject_to(X_(x_id, all) <= ub_[x_id]);
        ocp.subject_to(X_(x_id, all) >= lb_[x_id]);
    }
}
