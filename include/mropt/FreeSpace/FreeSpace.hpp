#ifndef FREESPACE_H
#define FREESPACE_H
#pragma once

#include "mropt/StateSpace/State.hpp"
#include "mropt/RobotShape/Footprint.h"
#include <casadi/casadi.hpp>
#include <boost/functional/hash.hpp>

using namespace casadi;

namespace mropt::freespace {
class FreeSpace {
public:
  struct PolygonAssignment {
    int k0;
    int kf;
    int pid;
  };

  struct Polygon {
    Polygon(const MX &A, const MX &b) : A(A), b(b), hp(A.size1()) {
      id_ = counter;
      ++counter;
    }
    static int counter;
    int id_;
    std::size_t hp;
    MX A;
    MX b;
  };

  FreeSpace(const FreeSpace &) = delete;
  FreeSpace &operator=(const FreeSpace &) = delete;

  static void add_polygon(MX &A, MX &b);
  static void add(std::list<Polygon> polys);
  static void clear_polygons();
  std::vector<MX> get_constraints(std::vector<PolygonAssignment> pas);
  casadi::DM J_real(const casadi::DM &x_r) {
    const auto &result = J_real_(std::vector<casadi::DM>{{x_r}});
    return result[0];
  }

  void setup(casadi::Opti &ocp) {}
  void setRobotShape(const std::shared_ptr<mropt::RobotShape::Footprint> &shape) { robot_shape = shape; }
  explicit FreeSpace(mropt::StateSpace::State &s) : ss(s) {}
  ~FreeSpace();
  static std::unordered_map<
            std::pair<int, int>,
            mropt::StateSpace::State::state,
            boost::hash<std::pair<int, int>> > *poly_centers;
private:
  friend class Robot;
  Function J_real_;
  Slice all;
  mropt::StateSpace::State &ss;
  static std::vector<Polygon> polygons;
  std::shared_ptr<mropt::RobotShape::Footprint> robot_shape;
  double threshold{0.0};
};
}
#endif

