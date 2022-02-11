#include <casadi/casadi.hpp>
#include <cmath>
#include <matplotlibcpp.h>
#include <string>

#include "mropt/StateSpace/SE2/SE2.hpp"
#include "mropt/ControlSpace/R2/VW.hpp"
#include "mropt/Dynamics/CarLike/DiffDrive.hpp"
#include "mropt/Dynamics/Approx/FirstOrderTaylor.hpp"
#include "mropt/Dynamics/Transcription/MultipleShooting.hpp"
#include "mropt/Dynamics/Transcription/MultipleShootingApprox.hpp"
#include "mropt/FreeSpace/FreeSpace.hpp"
#include "mropt/Cost/Cost.hpp"
#include "mropt/Collisions/L2norm/L2NormCollision.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorCollisions.h"
#include "mropt/Problem/Robot.hpp"
#include "mropt/Problem/CoupledProblem.hpp"
#include "mropt/RobotShape/CircleRobot.h"

#include "mropt/util/Opencv_plotter.hpp"



int main()
{
  std::string map_file = "c_map.png";
  std::string maps_path = "/home/ohmy/js_ws/github_joao/MROpt/maps/";

  int R = 4;

  //plotter.disp();

  int N = 40;
  double T = 50.0;
  //State Space
  auto se2_r1 = std::make_shared<SE2>();
  auto se2_r2 = std::make_shared<SE2>(); *se2_r2 = *se2_r1; //copy
  //Control Space
  auto vw_r1 = std::make_shared<VW>();
  auto vw_r2 = std::make_shared<VW>(); *vw_r2 = *vw_r1; //copy

  // Dynamic Model
  double L1 = 1.0, L2 = 1.0;
  auto model_r1 = std::make_shared<DiffDrive>(vw_r1, se2_r1, L1);
  auto model_r2 = std::make_shared<DiffDrive>(vw_r2, se2_r2, L2);

  //Robot Shape
  auto shape_r1 = std::make_shared<CircleRobot>(L1/2);
  auto shape_r2 = std::make_shared<CircleRobot>(L2/2);


  // Model Approx
  auto fot_r1 = std::make_shared<FirstOrderTaylor>(model_r1);
  auto fot_r2 = std::make_shared<FirstOrderTaylor>(model_r2);

  // Transcription Approx
  auto ms_approx_r1 = std::make_shared<MultipleShootingApprox>(fot_r1);
  auto ms_approx_r2 = std::make_shared<MultipleShootingApprox>(fot_r2);

  // Cost
  auto cost_r1 = std::make_shared<Cost>(*vw_r1, *se2_r1);
  auto cost_r2 = std::make_shared<Cost>(*vw_r2, *se2_r2);

  // Free Space
  auto cfree_r1 = std::make_shared<FreeSpace>(*se2_r1);
  auto cfree_r2 = std::make_shared<FreeSpace>(*se2_r2);

  //Normals Rectangle
  MX A = MX(4, 2);
  A = DM(
      {{-1, 0},
       {0, 1},
       {1, 0},
       {0, -1}});

  FreeSpace::Polygon P1{A, DM::vertcat({0, 3, 10, 0})};
  FreeSpace::Polygon P2{A, DM::vertcat({-7, 10, 10, 0})};
  FreeSpace::Polygon P3{A, DM::vertcat({0, 10, 10, -7})};
  cfree_r1->add({P1, P2, P3});
  for(int i = 0; i < 100; i++){
    FreeSpace::Polygon Pt{A, DM::vertcat({0, 10, 10, -7})};
    cfree_r1->add({std::move(Pt)});
  }
  FreeSpace::PolygonAssignment p1a{0, (int)N / 3 + 1, P1.id_};
  FreeSpace::PolygonAssignment p2a{(int)N / 3 , (int)2 * N / 3 + 1, P2.id_};
  FreeSpace::PolygonAssignment p3a{(int)2 * N / 3, N, P3.id_};
  FreeSpace::PolygonAssignment pr2{0, N, P3.id_};

  FreeSpace::poly_centers =
      new std::unordered_map<std::pair<int, int>, State::state, boost::hash<std::pair<int, int>>>
          {
              {{P1.id_,P2.id_}, {8.5, 1.5, M_PI/2}},
              {{P2.id_,P1.id_}, {8.5, 1.5, M_PI}},
              {{P2.id_,P3.id_}, {8.5, 8.5, M_PI}},
              {{P3.id_,P2.id_}, {8.5, 8.5, -M_PI/2}}
          };


  // Collisions
  auto collision_type = L2NormCollision();
  auto fot_col = std::make_shared<FirstOrderTaylorCollisions>( collision_type);

  // Problem
  auto params = Robot::Params{0.0, T, N};
  auto robot1 = std::make_shared<Robot>(params, vw_r1, se2_r1, cost_r1, cfree_r1, ms_approx_r1, shape_r1);
  auto robot2 = std::make_shared<Robot>(params, vw_r2, se2_r2, cost_r2, cfree_r2, ms_approx_r2, shape_r2);

  // SingleMission
  double x_0{1.0}, y_0{2.0}, o_0{M_PI/2};
  double x_f{1.0}, y_f{8.0}, o_f{M_PI/2};
  robot1->addMission({x_0, y_0, o_0}, {x_f, y_f}, {p1a, p2a, p3a});
  robot2->addMission({x_f, y_f, o_0}, {8.0, y_f}, {pr2});


//  //Multiple robots
//  double L3 = 0.4;
//  auto se2_r3 = se2_r1; auto vw_r3 = vw_r1;
//  auto model_r3 = DiffDrive(vw_r3, se2_r3, L3);
//  auto shape_r3 = std::make_shared<CircleRobot>(L3/2);
//  auto fot_r3 = FirstOrderTaylor(model_r3);
//  auto ms_approx_r3 = MultipleShootingApprox(fot_r3);
//  auto cost_r3 = Cost(vw_r3, se2_r3);
//  auto cfree_r3 = FreeSpace(se2_r3);
//  auto robot3 = std::make_shared<Robot>(vw_r3, se2_r3, cost_r3, cfree_r3, ms_approx_r3, shape_r3, params);
//  robot3->addMission({3.0, 1.0, M_PI/2}, {3.0, 8.0}, {pmr1a, pmr2a, pmr3a });


  OptProblem mrprob;
mrprob.allow_plotting();
mrprob.addRobot(robot1);
mrprob.addRobot(robot2);
mrprob.addCollisions(fot_col);
mrprob.scp();


std::vector<double> x1, y1, o1; std::vector<std::vector<double>> u1;
robot1->get_solution(x1,y1,o1, u1);
std::vector<double> x2, y2, o2; std::vector<std::vector<double>> u2;
robot2->get_solution(x2,y2,o2, u2);

robot1->plot();
robot2->plot();



  Opencv_plotter plotter({model_r1, model_r2}); //TODO: shared_ptr
  plotter.setFootprint({L1, L2}, {0, 0});
  plotter.addPathToScenarios(maps_path);
  plotter.inputScenario(map_file);

plotter.plot_trajectory({x1,x2}, {y1, y2}, {o1, o2}, {u1, u2}, T);

}
































































// int main()
// {
//   casadi::Opti ocp("conic");
//   int N = 50;
//   double T = 30.0;
//   double nu = N;
//   double nx = N + 1;

//   //State Space
//   double w_x = 0;
//   double w_y = 0;
//   double w_o = 0.1;
//   double x_std = 0.0;
//   double y_std = 0.0;
//   double o_std = 0.0;
//   auto se2 = SE2().set_weights_std_values({w_x, w_y, w_o}, {x_std, y_std, o_std});
//   double scn_max_x = 10.0;
//   double scn_max_y = 10.0;
//   se2.set_lower_bounds({0.0, 0.0});             //left corner xy
//   se2.set_upper_bounds({scn_max_x, scn_max_y}); //upper corner xy
//   //auto g_state = se2.get_constraints();

//   //Control Space
//   double w_u_v = 1;
//   double w_u_w = 2;
//   double u_v_std = 0.5;
//   double u_w_std = 0.0;
//   auto vw = VW().set_weights_std_values({w_u_v, w_u_w}, {u_v_std, u_w_std});
//   double b_u_v = 1.0;
//   double b_u_w = 2.0;
//   vw.set_lower_bounds({-b_u_v, -b_u_w}); //left corner xy
//   vw.set_upper_bounds({b_u_v, b_u_w});   //upper corner xy
//   //auto g_control = vw.get_constraints();

//   // Dynamic Model
//   double L = 0.5;
//   auto model = DiffDrive(vw, se2, L);

//   // Model Approx
//   auto fot = FirstOrderTaylorCollisions(model);

//   // Transcription
//   double t0 = 0.0;
//   double tf = 30.0;
//   // auto ms = MultipleShooting(N, t0, tf, model);
//   // auto g_ms = ms.get_constraints();

//   // Transcription Approx
//   auto ms_approx = MultipleShootingApprox(fot);
//   // Transcription No Approx
//   auto ms = MultipleShooting(fot);
//   // Cost
//   auto cost = Cost(vw, se2);
//   //auto l2norm_cost = cost.integrated_cost(t0, tf, N);

//   // Free Space
//   auto cfree = FreeSpace(se2);
//   //Normals Rectangle
//   MX A = MX(4, 2);
//   A = DM(
//       {{-1, 0},
//        {0, 1},
//        {1, 0},
//        {0, -1}});

//   FreeSpace::Polygon P1{A, DM::vertcat({-0, 3, 10, -0})};
//   FreeSpace::Polygon P2{A, DM::vertcat({-7, 10, 10, -0})};
//   FreeSpace::Polygon P3{A, DM::vertcat({-0, 10, 10, -7})};
//   cfree.add({P1, P2, P3});
//   FreeSpace::PolygonAssignment p1a{0, (int)N / 3, P1.id_};
//   FreeSpace::PolygonAssignment p2a{(int)N / 3, (int)2 * N / 3, P2.id_};
//   FreeSpace::PolygonAssignment p3a{(int)2 * N / 3, N + 1, P3.id_};
//   //auto g_cfree = cfree.get_constraints( {p1a, p2a, p3a} );

//   // Problem
//   auto params = Problem::Params{0.0, T, N};
//   auto mrprob = Problem(vw, se2, cost, cfree, ms, params);

//   // SingleMission
//   // auto mission = SingleMission(se2);
//   double x_0{0.5}, y_0{0.5}, o_0{0};
//   double x_f{0.5}, y_f{9.5}, o_f{M_PI / 2};
//   // mission.init({x_0, y_0, o_0});
//   // mission.goal({x_f, y_f, o_f});
//   // auto g_mission = mission.get_constraints();

//   mrprob.addMission({x_0, y_0, o_0}, {x_f, y_f, o_f});

//   auto x_init = std::vector(N + 1, 0.0);
//   auto y_init = std::vector(N + 1, 0.0);
//   auto o_init = std::vector(N + 1, 0.0);
//   auto u_v_init = std::vector(N, u_v_std);
//   auto u_w_init = std::vector(N, 0.0);

//   DM X0 = DM({x_init, y_init, o_init});
//   DM U0 = DM({u_v_init, u_w_init});

//   mrprob.solve(X0, U0);

//   mrprob.plot();
// }







// using namespace casadi;
// namespace plt = matplotlibcpp;
// MX integrate_rk4(Function &f, const MX &dt, const MX &x, const MX &u)
// {
//   auto k1 = f({{x}, {u}});
//   auto k2 = f({{x + ((double)1 / 2) * dt * k1[0]}, {u}});
//   auto k3 = f({{x + ((double)1 / 2) * dt * k2[0]}, {u}});
//   auto k4 = f({{x + dt * k3[0]}, {u}});
//   return ((double)1 / 6) * dt * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
// }

// std::vector<double> LinearSpacedVector(double a, double b, std::size_t N)
// {
//   double h = (b - a) / static_cast<double>(N - 1);
//   std::vector<double> xs(N);
//   std::vector<double>::iterator x;
//   double val;
//   for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
//   {
//     *x = val;
//   }
//   return xs;
// }

// void convexify_dynamics(
//     Opti &ocp,
//     MX &f_x0u0, std::vector<MX> &jac_f_x0u0, MX &u0, MX &x0,
//     const Function &f, const Function &jac_f,
//     DM &X0, DM &U0)
// {
//   Slice all;
//   auto nu = u0.size1();
//   auto nx = x0.size1();
//   auto N = u0.size2();

//   ocp.set_value(x0, X0);
//   ocp.set_value(u0, U0);

//   DM f_x0u0_val = DM(nx, N + 1);
//   std::vector<DM> jac_f_x0u0_val(N + 1, DM(nx, nx + nu));
//   for (int k = 0; k < N; ++k)
//   {
//     f_x0u0_val(all, k) = f(std::vector<casadi::DM>{{X0(all, k)}, {U0(all, k)}})[0];
//     jac_f_x0u0_val[k] = jac_f(std::vector<casadi::DM>{{X0(all, k)}, {U0(all, k)}, {}})[0];
//     ocp.set_value(jac_f_x0u0[k], jac_f_x0u0_val[k]);
//   }
//   ocp.set_value(f_x0u0, f_x0u0_val);
// }

// int main()
// {

//   casadi::Opti ocp("conic");

//   int scn_max_x = 10;
//   int scn_max_y = 10;
//   //Decision Variables
//   Slice all;
//   int N = 50;
//   double T = 30.0;
//   double nu = N;
//   double nx = N + 1;
//   auto X = ocp.variable(3, nx);
//   auto U = ocp.variable(2, nu);
//   auto x = X(0, all);
//   auto y = X(1, all);
//   auto xy = MX::vertcat({x, y});
//   auto o = X(2, all);
//   auto u_v = U(0, all);
//   auto u_w = U(1, all);

//   //ODE vars
//   auto t = ocp.parameter(1);
//   auto dt = t / N;
//   SX L = 0.5;
//   SX x_ode = SX::sym("x");
//   SX y_ode = SX::sym("y");
//   SX o_ode = SX::sym("o");
//   SX u_v_ode = SX::sym("u_v");
//   SX u_w_ode = SX::sym("u_w");
//   SX X_ode = SX::vertcat({x_ode, y_ode, o_ode});
//   SX U_ode = SX::vertcat({u_v_ode, u_w_ode});
//   //x_dot = f(x,u);
//   SX X_dot = SX::vertcat(
//       {u_v_ode * cos(o_ode),
//        u_v_ode * sin(o_ode),
//        (1 / L) * u_w_ode});

//   //Cost
//   double w_x = 0;
//   double w_y = 0;
//   double w_o = 0.1;
//   double w_u_v = 1;
//   double w_u_w = 2;
//   SX R = DM::diag({w_u_v, w_u_w});
//   SX Q = DM::diag({w_x, w_y, w_o});
//   SX u_v_std = 0.5;
//   SX u_w_std = 0.0;
//   SX U_std = SX::vertcat({u_v_std, u_w_std});

//   SX cost =
//       mtimes(transpose(U_ode) - transpose(U_std), mtimes(R, U_ode - U_std)) +
//       mtimes(transpose(X_ode), mtimes(Q, X_ode));

//   //Dynamic Function
//   auto f = Function("f", {X_ode, U_ode}, {X_dot});
//   //Cost Function
//   auto l = Function("l", {X_ode, U_ode}, {cost});

//   //Integrated cost
//   MX J = 0;
//   for (int k = 0; k < N; ++k)
//   {
//     //MX X = MX::vertcat( {X(all,k)});
//     J = J + integrate_rk4(l, dt, X(all, k), U(all, k));
//   }
//   ocp.minimize(J);

//   auto jac_f = f.jacobian();

//   // std::cout << f(std::vector<casadi::DM>{{1, 2, 3}, {1, 2}}) << std::endl;
//   // std::cout << jac_f(std::vector<casadi::DM>{{1, 2, M_PI / 4}, {3, 6}, {}}) << std::endl;

//   // --- Bounds ---
//   ocp.bounded(-1, u_v(all), 1);
//   ocp.bounded(-2, u_w(all), 2);
//   // ocp.subject_to(0 <= x(all) <= 10);
//   // ocp.subject_to(0 <= y(all) <= 10);
//   //ocp.subject_to(-M_PI <= o(all) <= M_PI);

//   //Polygons
//   struct Polygon
//   {
//     Polygon(MX A, MX b) : A(A), b(b)
//     {
//       hp = A.size1();
//     }
//     std::size_t hp;
//     MX A;
//     MX b;
//   };
//   //Normals Rectangle
//   MX A = MX(4, 2);
//   A = DM(
//       {{-1, 0},
//        {0, 1},
//        {1, 0},
//        {0, -1}});
//   std::cout << A << std::endl;
//   std::cout << A.size2() << std::endl;
//   int P = 3;
//   Polygon P1{A, DM::vertcat({-0, 3, 10, -0})};
//   Polygon P2{A, DM::vertcat({-7, 10, 10, -0})};
//   Polygon P3{A, DM::vertcat({-0, 10, 10, -7})};
//   std::vector<Polygon> polygons{P1, P2, P3};

//   //Assignment Matrix states to polygons
//   auto M_px = ocp.parameter(P, N + 1);

//   for (int p = 0; p < P; ++p)
//   {
//     auto &polygon = polygons[p];
//     auto g_p1 = mtimes(polygon.A, xy) - repmat(polygon.b, 1, N + 1) - mtimes(MX::ones(polygon.hp, 1), M_px(p, all));
//     ocp.subject_to(g_p1(all) <= 0);
//   }

//   //ocp.set_value(M_px,)
//   //ocp.set_value(M_px , DM::zeros(P,N+1));
//   Slice p1slice(0, (int)N / 3);
//   Slice p2slice((int)N / 3, (int)2 * N / 3);
//   Slice p3slice((int)2 * N / 3, N + 1);

//   ocp.set_value(M_px(all), 1.1 * 100 * (scn_max_x + scn_max_y));
//   ocp.set_value(M_px(0, p1slice), 0);
//   ocp.set_value(M_px(1, p2slice), 0);
//   ocp.set_value(M_px(2, p3slice), 0);

//   // ---- Initial conditions --------
//   double x_0{0.5}, y_0{0.5}, o_0{0};
//   double x_f{0.5}, y_f{9.5}, o_f{M_PI / 2};
//   ocp.subject_to(x(0) == x_0); // start at position 0 ...
//   ocp.subject_to(y(0) == y_0); // ... from stand-still
//   ocp.subject_to(o(0) == o_0); // finish line at position 1
//   // ---- Final conditions --------
//   ocp.subject_to(x(N) == x_f); // start at position 0 ...
//   ocp.subject_to(y(N) == y_f); // ... from stand-still
//   ocp.subject_to(o(N) == o_f); // finish line at position 1

//   // --- Initial Guess ---
//   std::vector<double> x_init;
//   std::vector<double> x_1 = LinearSpacedVector(x_0, 8.5, p1slice.size());
//   std::vector<double> x_2 = LinearSpacedVector(8.5, 8.5, p2slice.size());
//   std::vector<double> x_3 = LinearSpacedVector(8.5, x_f, p3slice.size());
//   x_init.insert(x_init.end(), x_1.begin(), x_1.end());
//   x_init.insert(x_init.end(), x_2.begin(), x_2.end());
//   x_init.insert(x_init.end(), x_3.begin(), x_3.end());

//   std::vector<double> y_init;
//   std::vector<double> y_1 = LinearSpacedVector(y_0, 1.5, p1slice.size());
//   std::vector<double> y_2 = LinearSpacedVector(1.5, 8.5, p2slice.size());
//   std::vector<double> y_3 = LinearSpacedVector(8.5, y_f, p3slice.size());
//   y_init.insert(y_init.end(), y_1.begin(), y_1.end());
//   y_init.insert(y_init.end(), y_2.begin(), y_2.end());
//   y_init.insert(y_init.end(), y_3.begin(), y_3.end());

//   std::vector<double> o_init;
//   std::vector<double> o_1 = LinearSpacedVector(o_0, 0.0, p1slice.size());
//   std::vector<double> o_2 = LinearSpacedVector(0.0, M_PI, p2slice.size());
//   std::vector<double> o_3 = LinearSpacedVector(M_PI, o_f, p3slice.size());

//   std::vector<double> u_v_init(N, 0.5);
//   std::vector<double> u_w_init(N, 0);

//   o_init.insert(o_init.end(), o_1.begin(), o_1.end());
//   o_init.insert(o_init.end(), o_2.begin(), o_2.end());
//   o_init.insert(o_init.end(), o_3.begin(), o_3.end());

//   ocp.set_initial(x(all), x_init);
//   ocp.set_initial(y(all), y_init);
//   ocp.set_initial(o(all), o_init);
//   ocp.set_initial(u_v(all), u_v_init);
//   ocp.set_initial(u_w(all), u_w_init);

//   DM X0 = DM({x_init, y_init, o_init});
//   DM U0 = DM({u_v_init, u_w_init});

//   //DM X0 = DM::vertcat({x_init.}, {y_init}, {o_init});

//   // ocp.set_initial(x(all), std::vector(N + 1, 0));
//   // ocp.set_initial(y(all), std::vector(N + 1, 0));
//   // ocp.set_initial(o(all), std::vector(N + 1, 0));
//   // ocp.set_initial(u_v(all), std::vector(N, u_v_std));
//   // ocp.set_initial(u_w(all), std::vector(N, 0));

//   //Dynamics approx
//   auto f_x0u0 = ocp.parameter(X_ode.size1(), N + 1);
//   std::vector<MX> jac_f_x0u0{};
//   for (int i = 0; i < N + 1; ++i)
//   {
//     jac_f_x0u0.push_back(ocp.parameter(X_dot.size1(), X_ode.size1() + U_ode.size1()));
//   }
//   auto x0 = ocp.parameter(X_ode.size1(), nx);
//   auto u0 = ocp.parameter(U_ode.size1(), nu);

//   // ---- dynamic constraints --------
//   auto x_ode_mx = MX::sym("x", 3);
//   auto u_ode_mx = MX::sym("u", 2);

//   for (int k = 0; k < N; ++k)
//   {
//     MX x_dot_taylor = f_x0u0(k) + mtimes(jac_f_x0u0[k], MX::vertcat({x_ode_mx - x0(all, k), u_ode_mx - u0(all, k)}));
//     auto f_approx = Function("f_approx", {{x_ode_mx}, {u_ode_mx}}, {x_dot_taylor});
//     auto x_next = X(all, k) + integrate_rk4(f_approx, dt, X(all, k), U(all, k));
//     ocp.subject_to(X(all, k + 1) == x_next); //close the gaps
//   }

//   // --- Set parameters ---
//   ocp.set_value(t, T);

//   //ocp.subject_to(A*X <= b)

//   // --- Prolem and Solver Options
//   std::string solver{"gurobi"};

//   Dict p_opts, s_opts;
//   p_opts["expand"] = true;
//   //s_opts["fixed_variable_treatment"] = "make_constraint";
//   Dict solver_options;
//   if (solver == "qpoases")
//   {
//     s_opts["sparse"] = true;
//     s_opts["schur"] = "schur";
//     s_opts["print_time"] = true;
//   }
//   // --- Solver ---
//   ocp.solver(solver, p_opts, s_opts);

//   int I = 20;
//   std::vector<double> x_sol, y_sol, o_sol, u_v_sol, u_w_sol;
//   for (int it = 0; it < I; ++it)
//   {
//     convexify_dynamics(ocp, f_x0u0, jac_f_x0u0, u0, x0, f, jac_f, X0, U0);
//     auto solution = ocp.solve();
//     x_sol = std::vector<double>(solution.value(x));
//     y_sol = std::vector<double>(solution.value(y));
//     o_sol = std::vector<double>(solution.value(o));
//     u_v_sol = std::vector<double>(solution.value(u_v));
//     u_w_sol = std::vector<double>(solution.value(u_w));
//     X0 = DM({x_sol, y_sol, o_sol});
//     U0 = DM({u_v_sol, u_w_sol});

//   }

//   //Solution and Plotting

//   // ocp.debug().show_infeasibilities();
//   //auto Jac_g = solution.value(jacobian(ocp.g(), ocp.x()));
//   //std::cout << Jac_g << std::endl;
//   //solution.disp(std::cout, true);
//   // auto x_sol = std::vector<double>(solution.value(x));
//   // auto y_sol = std::vector<double>(solution.value(y));
//   // auto o_sol = std::vector<double>(solution.value(o));
//   // auto u_v_sol = solution.value(u_v);
//   // auto u_w_sol = solution.value(u_w);
//   //std::cout << solution.stats(); // dict

//   std::vector<double> o_sol_x = o_sol;
//   std::vector<double> o_sol_y = o_sol;
//   std::for_each(
//       o_sol_x.begin(),
//       o_sol_x.end(),
//       [](double &o) {
//         o = std::cos(o);
//       });
//   std::for_each(
//       o_sol_y.begin(),
//       o_sol_y.end(),
//       [](double &o) {
//         o = std::sin(o);
//       });
//   auto up = y_sol;
//   std::for_each(
//       up.begin(),
//       up.end(),
//       [](double &y) {
//         y += 0.5;
//       });
//   auto down = y_sol;
//   std::for_each(
//       down.begin(),
//       down.end(),
//       [](double &y) {
//         y -= 0.5;
//       });

//   std::vector<double> x_s, y_s, o_s_x, o_s_y, up_s, down_s;
//   std::vector<double> car_xl, car_yl, car_xr, car_yr;
//   for (int i = 0; i < o_sol_x.size(); ++i)
//   {
//     plt::clf();
//     x_s.push_back(x_sol[i]);
//     y_s.push_back(y_sol[i]);
//     o_s_x.push_back(o_sol_x[i]);
//     o_s_y.push_back(o_sol_y[i]);
//     up_s.push_back(up[i]);
//     down_s.push_back(down[i]);

//     double x_w_l = x_sol[i] - 0.5 * sin(o_sol[i]);
//     double y_w_l = y_sol[i] + 0.5 * cos(o_sol[i]);
//     double x_w_r = x_sol[i] + 0.5 * sin(o_sol[i]);
//     double y_w_r = y_sol[i] - 0.5 * cos(o_sol[i]);
//     car_xl.push_back(x_w_l);
//     car_xr.push_back(x_w_r);
//     car_yl.push_back(y_w_l);
//     car_yr.push_back(y_w_r);
//     plt::quiver(x_s, y_s, o_s_x, o_s_y);
//     //plt::plot(x_sol,  y_sol);
//     plt::plot(car_xl, car_yl);
//     plt::plot(car_xr, car_yr);
//     plt::xlim(0, 10);
//     plt::ylim(0, 10);
//     plt::axis("equal");

//     std::map<std::string, std::string> keywords;
//     keywords["alpha"] = "0.4";
//     keywords["color"] = "grey";
//     //keywords["hatch"] = "-";
//     //plt::fill_between(x_s, y_s, up_s, keywords);
//     //plt::fill_between(x_s, y_s, down_s, keywords);
//     plt::pause(0.001);
//   }

//   plt::show();
// }

// #include <casadi/casadi.hpp>
// #include <math.h>
// #include <matplotlibcpp.h>
// #include <map>
// #include <string>
// #include <limits>

// using namespace casadi;
// namespace plt = matplotlibcpp;

// MX integrate_rk4(Function& f, const MX& dt, const MX& x, const MX& u){
//     auto k1 = f({{x},{u}});
//     auto k2 = f({{x + ((double)1 / 2) * dt * k1[0]}, {u}});
//     auto k3 = f({{x + ((double)1 / 2) * dt * k2[0]}, {u}});
//     auto k4 = f({{x + dt * k3[0]}, {u}});
//     return ((double)1 / 6) * dt * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
// }

// std::vector<double> LinearSpacedVector(double a, double b, std::size_t N)
// {
//   double h = (b - a) / static_cast<double>(N - 1);
//   std::vector<double> xs(N);
//   std::vector<double>::iterator x;
//   double val;
//   for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
//   {
//     *x = val;
//   }
//   return xs;
// }

// int main()
// {

//   casadi::Opti ocp;

//   int scn_max_x = 10;
//   int scn_max_y = 10;
//   //Decision Variables
//   Slice all;
//   int N = 100;

//   double T = 30.0;
//   double nu = N;
//   double nx = N + 1;
//   auto X = ocp.variable(3, nx);
//   auto U = ocp.variable(2, nu);
//   auto x = X(0, all);
//   auto y = X(1, all);
//   auto xy = MX::vertcat({x, y});
//   auto o = X(2, all);
//   auto u_v = U(0, all);
//   auto u_w = U(1, all);

//   //ODE vars
//   auto t = ocp.parameter(1);
//   SX L = 0.5;
//   SX x_ode = SX::sym("x");
//   SX y_ode = SX::sym("y");
//   SX o_ode = SX::sym("o");
//   SX u_v_ode = SX::sym("u_v");
//   SX u_w_ode = SX::sym("u_w");
//   SX X_ode = SX::vertcat({x_ode, y_ode, o_ode});
//   SX U_ode = SX::vertcat({u_v_ode, u_w_ode});
//   //x_dot = f(x,u);
//   SX X_dot = SX::vertcat(
//       {u_v_ode * cos(o_ode),
//        u_v_ode * sin(o_ode),
//        (1 / L) * u_w_ode});
//   //Cost
//   double w_x = 0;
//   double w_y = 0;
//   double w_o = 10;
//   double w_u_v = 1;
//   double w_u_w = 2;
//   SX R = DM::diag({w_u_v, w_u_w});
//   SX Q = DM::diag({w_x, w_y, w_o});
//   SX u_v_std = 0.5;
//   SX u_w_std = 0.0;
//   SX U_std = SX::vertcat({u_v_std, u_w_std});

//   SX cost =
//       mtimes(transpose(U_ode) - transpose(U_std), mtimes(R, U_ode - U_std)) +
//       mtimes(transpose(X_ode), mtimes(Q, X_ode));

//   int a = mtimes(R, U_ode).size1();
//   std::cout << a << std::endl;
//   std::cout << R << std::endl;

//   //Dynamic Function
//   auto f = Function("f", {X_ode, U_ode}, {X_dot});
//   //Cost Function
//   auto l = Function("l", {X_ode, U_ode}, {cost});

//   auto jac_f = f.jacobian();
//   auto f_p = Function("f", {X_ode, U_ode}, {X_dot});

//   // ---- dynamic constraints --------
//   auto dt = t / N;
//   for (int k = 0; k < N; ++k)
//   {
//     auto x_next = X(all, k) + integrate_rk4(f, dt, X(all, k), U(all, k));
//     //std::cout << x_next << std::endl;
//     ocp.subject_to(X(all, k + 1) == x_next); //close the gaps
//   }

//   //Integrated cost
//   MX J = 0;
//   for (int k = 0; k < N; ++k)
//   {
//     //MX X = MX::vertcat( {X(all,k)});
//     J = J + integrate_rk4(l, dt,  X(all, k), U(all, k));
//   }
//   // MX J = 0;
//   // for (int k = 0; k < N; ++k)
//   // {
//   //   J += J + dt * l({U(all, k)})[0];
//   // }
//   //std::cout << J << std::endl;
//   ocp.minimize(J);

//   // --- Bounds ---
//   ocp.bounded(-1, u_v(all), 1);
//   ocp.bounded(-2, u_w(all), 2);
//   // ocp.subject_to(0 <= x(all) <= 10);
//   // ocp.subject_to(0 <= y(all) <= 10);
//   //ocp.subject_to(-M_PI <= o(all) <= M_PI);

//   //Polygons
//   struct Polygon
//   {
//     Polygon(MX A, MX b) : A(A), b(b)
//     {
//       hp = A.size1();
//     }
//     std::size_t hp;
//     MX A;
//     MX b;
//   };
//   //Normals Rectangle
//   MX A = MX(4, 2);
//   A = DM(
//       {{-1, 0},
//        {0, 1},
//        {1, 0},
//        {0, -1}});
//   std::cout << A << std::endl;
//   std::cout << A.size2() << std::endl;
//   int P = 3;
//   Polygon P1{A, DM::vertcat({-0, 3, 10, -0})};
//   Polygon P2{A, DM::vertcat({-7, 10, 10, -0})};
//   Polygon P3{A, DM::vertcat({-0, 10, 10, -7})};
//   std::vector<Polygon> polygons{P1, P2, P3};

//   //Assignment Matrix states to polygons
//   auto M_px = ocp.parameter(P, N + 1);

//   for (int p = 0; p < P; ++p)
//   {
//     auto &polygon = polygons[p];
//     auto g_p1 = mtimes(polygon.A, xy) - repmat(polygon.b, 1, N + 1) - mtimes(MX::ones(polygon.hp, 1), M_px(p, all));
//     ocp.subject_to(g_p1(all) <= 0);
//   }

//   //ocp.set_value(M_px,)
//   //ocp.set_value(M_px , DM::zeros(P,N+1));
//   Slice p1slice(0, (int)N / 3);
//   Slice p2slice((int)N / 3, (int)2 * N / 3);
//   Slice p3slice((int)2 * N / 3, N + 1);

//   ocp.set_value(M_px(all), 1.1 * 100 * (scn_max_x + scn_max_y));
//   ocp.set_value(M_px(0, p1slice), 0);
//   ocp.set_value(M_px(1, p2slice), 0);
//   ocp.set_value(M_px(2, p3slice), 0);

//   // ---- Initial conditions --------
//   double x_0{0.5}, y_0{0.5}, o_0{0};
//   double x_f{0.5}, y_f{9.5}, o_f{M_PI/2};
//   ocp.subject_to(x(0) == x_0); // start at position 0 ...
//   ocp.subject_to(y(0) == y_0); // ... from stand-still
//   ocp.subject_to(o(0) == o_0); // finish line at position 1
//   // ---- Final conditions --------
//   ocp.subject_to(x(N) == x_f); // start at position 0 ...
//   ocp.subject_to(y(N) == y_f); // ... from stand-still
//   //ocp.subject_to(o(N) == o_f); // finish line at position 1
//   auto g_o_s = sin(o(N)) - sin(o_f);
//   auto g_o_c = cos(o(N)) - cos(o_f);
//   ocp.subject_to(g_o_s == 0); // finish line at position 1
//   ocp.subject_to(g_o_c == 0); // finish line at position 1
//   // --- Initial Guess ---
//   std::vector<double> x_init;
//   std::vector<double> x_1 = LinearSpacedVector(x_0, 8.5, p1slice.size());
//   std::vector<double> x_2 = LinearSpacedVector(8.5, 8.5, p2slice.size());
//   std::vector<double> x_3 = LinearSpacedVector(8.5, x_f, p3slice.size());
//   x_init.insert(x_init.end(), x_1.begin(), x_1.end());
//   x_init.insert(x_init.end(), x_2.begin(), x_2.end());
//   x_init.insert(x_init.end(), x_3.begin(), x_3.end());

//   std::vector<double> y_init;
//   std::vector<double> y_1 = LinearSpacedVector(y_0, 1.5, p1slice.size());
//   std::vector<double> y_2 = LinearSpacedVector(1.5, 8.5, p2slice.size());
//   std::vector<double> y_3 = LinearSpacedVector(8.5, y_f, p3slice.size());
//   y_init.insert(y_init.end(), y_1.begin(), y_1.end());
//   y_init.insert(y_init.end(), y_2.begin(), y_2.end());
//   y_init.insert(y_init.end(), y_3.begin(), y_3.end());

//   std::vector<double> o_init;
//   std::vector<double> o_1 = LinearSpacedVector(o_0, 0.0, p1slice.size());
//   std::vector<double> o_2 = LinearSpacedVector(0.0, M_PI, p2slice.size());
//   std::vector<double> o_3 = LinearSpacedVector(M_PI, o_f, p3slice.size());

//   o_init.insert(o_init.end(), o_1.begin(), o_1.end());
//   o_init.insert(o_init.end(), o_2.begin(), o_2.end());
//   o_init.insert(o_init.end(), o_3.begin(), o_3.end());

//   ocp.set_initial(x(all), x_init);
//   ocp.set_initial(y(all), y_init);
//   ocp.set_initial(o(all), o_init);
//   ocp.set_initial(u_v(all), std::vector(N, u_v_std));
//   ocp.set_initial(u_w(all), std::vector(N, 0));

//   // ocp.set_initial(x(all), std::vector(N + 1, 0));
//   // ocp.set_initial(y(all), std::vector(N + 1, 0));
//   //ocp.set_initial(o(all), std::vector(N + 1, 0));
//   // ocp.set_initial(u_v(all), std::vector(N, u_v_std));
//   // ocp.set_initial(u_w(all), std::vector(N, 0));

//   // --- Set parameters ---
//   ocp.set_value(t, T);

//   //ocp.subject_to(A*X <= b)

//   // --- Prolem and Solver Options
//   Dict p_opts, s_opts;
//   p_opts["expand"] = true;
//   //s_opts["fixed_variable_treatment"] = "make_constraint";

//   // --- Solver ---
//   ocp.solver("ipopt", p_opts, s_opts);

//   auto solution = ocp.solve();

//   // ocp.debug().show_infeasibilities();
//   //auto Jac_g = solution.value(jacobian(ocp.g(), ocp.x()));
//   //std::cout << Jac_g << std::endl;
//   //solution.disp(std::cout, true);
//   auto x_sol = std::vector<double>(solution.value(x));
//   auto y_sol = std::vector<double>(solution.value(y));
//   auto o_sol = std::vector<double>(solution.value(o));
//   auto u_v_sol = solution.value(u_v);
//   auto u_w_sol = solution.value(u_w);
//   //std::cout << solution.stats(); // dict

//   std::vector<double> o_sol_x = o_sol;
//   std::vector<double> o_sol_y = o_sol;
//   std::for_each(
//       o_sol_x.begin(),
//       o_sol_x.end(),
//       [](double &o) {
//         o = std::cos(o);
//       });
//   std::for_each(
//       o_sol_y.begin(),
//       o_sol_y.end(),
//       [](double &o) {
//         o = std::sin(o);
//       });
//   auto up = y_sol;
//   std::for_each(
//       up.begin(),
//       up.end(),
//       [](double &y) {
//         y += 0.5;
//       });
//   auto down = y_sol;
//   std::for_each(
//       down.begin(),
//       down.end(),
//       [](double &y) {
//         y -= 0.5;
//       });

//   std::vector<double> x_s, y_s, o_s_x, o_s_y, up_s, down_s;
//   std::vector<double> car_xl, car_yl, car_xr, car_yr;
//   for (int i = 0; i < o_sol_x.size(); ++i)
//   {
//     plt::clf();
//     x_s.push_back(x_sol[i]);
//     y_s.push_back(y_sol[i]);
//     o_s_x.push_back(o_sol_x[i]);
//     o_s_y.push_back(o_sol_y[i]);
//     up_s.push_back(up[i]);
//     down_s.push_back(down[i]);

//     double x_w_l = x_sol[i] - 0.5 * sin(o_sol[i]);
//     double y_w_l = y_sol[i] + 0.5 * cos(o_sol[i]);
//     double x_w_r = x_sol[i] + 0.5 * sin(o_sol[i]);
//     double y_w_r = y_sol[i] - 0.5 * cos(o_sol[i]);
//     car_xl.push_back(x_w_l);
//     car_xr.push_back(x_w_r);
//     car_yl.push_back(y_w_l);
//     car_yr.push_back(y_w_r);
//     plt::quiver(x_s, y_s, o_s_x, o_s_y);
//     //plt::plot(x_sol,  y_sol);
//     plt::plot(car_xl, car_yl);
//     plt::plot(car_xr, car_yr);
//     plt::xlim(0, 10);
//     plt::ylim(0, 10);
//     plt::axis("equal");

//     std::map<std::string, std::string> keywords;
//     keywords["alpha"] = "0.4";
//     keywords["color"] = "grey";
//     //keywords["hatch"] = "-";
//     //plt::fill_between(x_s, y_s, up_s, keywords);
//     //plt::fill_between(x_s, y_s, down_s, keywords);
//     plt::pause(0.001);
//   }

//   plt::show();

// }

// SXDict dae = {{"x", X_ode}, {"p", U_ode}, {"ode", ode}, {"quad", cost}};
// Dict opts;
// opts["fsens_err_con"] = true;
// opts["quad_err_con"] = true;
// opts["abstol"] = 1e-6;
// opts["reltol"] = 1e-6;
// opts["stop_at_end"] = false;
// opts["linear_solver"] = "csparse";
// //  opts["fsens_all_at_once"] = false;
// opts["steps_per_checkpoint"] = 100;
// // opts["t0"] = 0;
// // opts["tf"] = T/nu;

// Function F = integrator("integrator", "cvodes", dae, opts);
// MX J = 0;
// for (int k = 0; k < N; ++k)
// {
//   // Create an evaluation node
//   MXDict integration =
//       F(
//           MXDict{
//               {"x0", MX::vertcat({x(k), y(k), o(k)})},
//               {"p", MX::vertcat({u_v(k), u_w(k)})}
//               }
//               );

//   ocp.subject_to(integration.at("xf") == X(all, k + 1));
//   // // Save continuity constraints
//   // g.push_back( I_out.at("xf") - X[k+1] );

//   // // Add objective function contribution
//   // J += I_out.at("qf");
//   J += mtimes(U(all, k).T(), U(all, k));
// }

// ocp.minimize(J);
// std::cout << ocp << std::endl;

//     casadi::Opti ocp;
//   casadi::DaeBuilder dae;

// double L = 0.8;
//   //auto L = dae.add_p("L");
//   //auto ti = dae.add_p("ti");
//   auto u_v = dae.add_u("u_v");
//   auto u_w = dae.add_u("u_w");
//   auto x = dae.add_x("x");
//   auto y = dae.add_x("y");
//   auto o = dae.add_x("o");

//   //ODE
//   auto x_dot = u_v * cos(o);
//   auto y_dot = u_v * sin(o);
//   auto o_dot = (1 / L) * u_w;
//   dae.add_ode("x_dot", x_dot);
//   dae.add_ode("y_dot", y_dot);
//   dae.add_ode("o_dot", o_dot);
//   //dae.add_lc("gamma", {"ode"});
//   auto ode = dae.create("f", {"x", "u", "p"}, {"ode"});
//   auto jac_ode_x = dae.create("ode", {"x", "u", "p"}, {"jac_ode_x"});
//   auto jac_ode_u = dae.create("ode", {"x", "u", "p"}, {"jac_ode_u"});
//   //auto hess_ode_x_x = dae.create("ode", {"x", "u", "p"}, {"hess_gamma_x_x"});

//   std::cout << ode <<std::endl;;
//   //Integrator
//     // Time length
//   double T = 10.0;

//   // Shooting length
//   int nu = 20; // Number of control segments

//   // Time horizon for integrator
//   double t0 = 0;
//   double tf = T/nu;

//   casadi::Dict opts;
//   // opts["expand_f"] = true;
//   // opts["interpolation_order"] = 1;
//   // opts["number_of_finite_elements"] = 1000;
//   opts["t0"] = t0;
//   opts["tf"] = tf;

//    //SXDict dae = {{"x", x}, {"p", U_ode}, {"t", t}, {"ode", ode}};dict
//   casadi::Function F = casadi::integrator("integrator", "rk", ode, opts);
