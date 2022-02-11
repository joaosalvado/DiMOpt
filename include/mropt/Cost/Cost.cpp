#include "Cost.hpp" 

using namespace mropt::cost;

SX Cost::generate_cost(){
		const auto &weight = SX::vertcat( {state_space_.get_weights(), control_space_.get_weights()} );
		const auto &vars_std = SX::vertcat( {state_space_.get_std_values(), control_space_.get_std_values()} );
		const auto &vars = casadi::SX::vertcat({state_space_.X_ode(), control_space_.U_ode()});
		if (weight.size2() != 1 || weight.size1() != vars.size1() || vars.size1() != vars_std.size1())
		{
			std::cerr << "[COST] weight, vars and vars_std vectors should agree" << std::endl;
		}
		auto P = SX::diag(weight);
		return mtimes(transpose(vars) - transpose(vars_std), mtimes(P, vars - vars_std));
	}	
