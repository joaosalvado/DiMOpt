//
// Created by ohmy on 2021-07-03.
//

#include <memory>
#include "CircleRobot.h"

using namespace mropt::RobotShape;

CircleRobot::CircleRobot(double R) : Footprint(), R(R) {}
double CircleRobot::get_safety_radius() {
  return R;
}
double* CircleRobot::get_safety_radius_ptr() {
  return &R;
}
std::shared_ptr <Footprint> CircleRobot::clone() const {
  return std::make_shared<CircleRobot>(*this);
}
