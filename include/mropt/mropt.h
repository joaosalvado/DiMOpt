//
// Created by ohmy on 2022-02-22.
//

#ifndef MROPT_MROPT_H
#define MROPT_MROPT_H

#include <casadi/casadi.hpp>
#include <cmath>
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
#include "mropt/Problem/DecoupledProblem.hpp"
#include "mropt/RobotShape/CircleRobot.h"
#include "mropt/util/Opencv_plotter.hpp"
#include "mropt/Collisions/Approx/FirstOrderTaylorDecoupledCollisions.h"
#include "mropt/Collisions/Approx/FirstOrderTaylorDistributedCollisions.h"
#include "mropt/Problem/DistributedRobot.h"

#include <mpi.h>

#endif //MROPT_MROPT_H
