#include "SingleMission.hpp"

using namespace mropt::Problem;

void SingleMission::init(std::vector<double> x)
{
    if (x.size() > ss.nx())
    {
        std::cerr << "[SingleMission] Size of the provided state does not match size of state space" << std::endl;
    }
    addNoise(x);
    x0 = std::move(x);
}

void SingleMission::goal(std::vector<double> x)
{
    if (x.size() > ss.nx())
    {
        std::cerr << "[SingleMission] Size of the provided state does not match size of state space" << std::endl;
    }
    addNoise(x);
    xf = std::move(x);
}

std::list<MX> SingleMission::get_constraints(){
    std::list<MX> constraints;
    // Init
    const auto X0 = ss.get_X_0();
    for( auto x_id = 0; x_id < x0.size(); ++ x_id){
        constraints.push_back( X0(x_id) - x0[x_id] );
    }

    const auto Xf = ss.get_X_f();
    for( auto x_id = 0; x_id < xf.size(); ++ x_id){
        constraints.push_back( Xf(x_id) - xf[x_id] );
    }

    return constraints;
}

void SingleMission::set_problem(MX &J_model, Opti& ocp){
  // Init
  const auto X0 = ss.get_X_0();
  for( auto x_id = 0; x_id < x0.size(); ++ x_id){
    ocp.subject_to( X0(x_id)-x0[x_id]==0);
  }


  const auto Xf = ss.get_X_f();
  for( auto x_id = 0; x_id < xf.size(); ++ x_id){
//    MX slack_var = ocp.variable(1);
    ocp.subject_to( Xf(x_id) - xf[x_id] == 0);
//    ocp.subject_to(slack_var >= 0);
//    J_model += slack_var;
  }
  //J_model += (Xf(0)-xf[0])*(Xf(0)-xf[0]) + (Xf(1)-xf[1])*(Xf(1)-xf[1]);

//  return constraints;
}

/// TODO: testing boundary constraints as cost function
/// \return
MX SingleMission::get_cost(){
  MX cost{0.0};
  const auto Xf = ss.get_X_f();
    for( auto x_id = 0; x_id < xf.size(); ++ x_id){
        cost = cost +( Xf(x_id) - xf[x_id] );
    }
  return cost;
}

void SingleMission::compute_polygon_assignments(){
  if(pas.empty()) return;
  auto Nx = pas[pas.size()-1].kf;
  pol_alloc = std::vector<std::vector<int>>(Nx, std::vector<int>{} );
  for(const auto& pa: pas){
    for(int k = pa.k0; k < pa.kf; ++k){
      pol_alloc[k].push_back(pa.pid);
    }
  }
}

void SingleMission::addNoise(std::vector<double> & x) {
  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double>
      distribution(0.001,0.02);
//  std::uniform_real_distribution<double>
//      distribution(0,0);

// Add only to x and y

  for(int i = 0; i < x.size(); i++){
      auto & x_i = x[i] ;
      if(i >= 2) continue;
    if(std::rand()%2 == 0) {
      x_i = x_i + distribution(generator);
    } else{
      x_i = x_i - distribution(generator);
    }
  }
}