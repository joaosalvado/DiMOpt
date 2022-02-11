#ifndef INTEGRATION_H
#define INTEGRATION_H
#pragma once

//#include <chrono>
#include <casadi/casadi.hpp>
using namespace casadi;
namespace mropt::util {

MX rk4(const Function &f, const MX &dt, const MX &x, const MX &u);
DM rk4_num(const Function &f, const DM &dt, const DM &x, const DM &u);

}
#endif