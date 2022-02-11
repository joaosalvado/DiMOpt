//
// Created by ohmy on 2021-07-03.
//

#ifndef MROPT_INCLUDE_MROPT_ROBOTSHAPE_FOOTPRINT_H
#define MROPT_INCLUDE_MROPT_ROBOTSHAPE_FOOTPRINT_H

#include <memory>

namespace mropt::RobotShape {
class Footprint {
public:
  Footprint() = default;
  virtual double get_safety_radius() = 0;
  virtual double *get_safety_radius_ptr() = 0;
  virtual ~Footprint() = default;
  virtual std::shared_ptr<Footprint> clone() const = 0;
};
}
#endif //MROPT_INCLUDE_MROPT_ROBOTSHAPE_FOOTPRINT_H
