#include "Control.hpp"

using namespace mropt::ControlSpace;

Control::~Control()
{
}

void Control::set_lower_bounds(std::vector<double> lb)
{
    if (lb.size() < nu())
    {
        std::cerr << "[State] lower bounds have lower size than state\n";
    }
    if (lb.size() > nu())
    {
        std::cerr << "[State] lower bounds have higher size than state\n";
        return;
    }
    lb_ = lb;
}

void Control::set_upper_bounds(std::vector<double> ub)
{
    if (ub.size() < nu())
    {
        std::cerr << "[State] upper bounds have lower size than state\n";
    }
    if (ub.size() > nu())
    {
        std::cerr << "[State] upper bounds have higher size than state\n";
        return;
    }
    ub_ = ub;
}

std::list<casadi::MX> Control::get_constraints()
{
    std::list<casadi::MX> constraints;
    for (int u_id = 0; u_id < lb_.size(); ++u_id)
    {
        constraints.push_back(-U_(u_id, all) + lb_[u_id]);
    }
    for (int u_id = 0; u_id < ub_.size(); ++u_id)
    {
        constraints.push_back(U_(u_id, all) - ub_[u_id]);
    }
    return constraints;
}

void Control::set_bounds(casadi::Opti &ocp)
{
    for (int u_id = 0; u_id < lb_.size(); ++u_id)
    {
        //ocp.bounded(lb_[u_id] , U_(u_id, all), ub_[u_id]);
        ocp.subject_to(U_(u_id, all) <= ub_[u_id]);
        ocp.subject_to(U_(u_id, all) >= lb_[u_id]);
    }
}


