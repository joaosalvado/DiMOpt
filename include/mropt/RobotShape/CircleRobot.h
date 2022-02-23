//
// Created by ohmy on 2021-07-03.
//

#ifndef MROPT_INCLUDE_MROPT_ROBOTSHAPE_CIRCLEROBOT_H
#define MROPT_INCLUDE_MROPT_ROBOTSHAPE_CIRCLEROBOT_H

#include "mropt/RobotShape/Footprint.h"

namespace mropt::RobotShape {
class CircleRobot : public Footprint {
private:
  double R;//radius
public:
  explicit CircleRobot(double R);
  virtual ~CircleRobot() = default;
  std::shared_ptr<Footprint> clone() const override;
  double getL() const { return 2*R;}
private:
  double get_safety_radius() override;
  double *get_safety_radius_ptr() override;
};
}

#endif //MROPT_INCLUDE_MROPT_ROBOTSHAPE_CIRCLEROBOT_H
