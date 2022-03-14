//
// Created by ohmy on 2022-03-14.
//

#include "AP.h"

using namespace mropt::ControlSpace;

std::shared_ptr<Control> AP::clone() const {
    return std::make_shared<AP>(*this);
}

AP& AP::set_weights_std_values(std::vector<double> weight, std::vector<double> vars_std)  {
    w_u_a = weight[0];
    w_u_p = weight[1];
    u_a_std = vars_std[0];
    u_p_std = vars_std[1];
    return *this;
}

AP::~AP()
{

}
