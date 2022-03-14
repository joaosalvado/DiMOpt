//
// Created by ohmy on 2022-03-14.
//

#include "SE2CU.h"

using namespace mropt::StateSpace;

std::shared_ptr<State> SE2CU::clone() const {
    return std::make_shared<SE2CU>(*this);
}
