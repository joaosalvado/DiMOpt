//
// Created by ohmy on 2022-03-24.
//

#ifndef MROPT_BUILDERROBOT_CGL_DUBINS_H
#define MROPT_BUILDERROBOT_CGL_DUBINS_H

#include "BuilderDistributedRobot_Dubins.h"
#include "mropt/mropt.h"
#include "mropt/Dynamics/Transcription/Pseudospectral/CGLms.h"

namespace mropt::Problem {
    class BuilderRobot_CGL_Dubins : public BuilderDistributedRobot_Dubins {
        int n; // degree of the Legendre polynomial
    public:
        BuilderRobot_CGL_Dubins(double L, int degree)
                : BuilderDistributedRobot_Dubins(L), n(degree) {}
        virtual ~BuilderRobot_CGL_Dubins() = default;
    private:
        std::shared_ptr<mropt::Dynamics::Transcription> build_transcription(
                const std::shared_ptr<mropt::Dynamics::OdeApprox> ode_approx,
                const std::shared_ptr<mropt::cost::Cost> cost) override;

    };
}



#endif //MROPT_BUILDERROBOT_CGL_DUBINS_H
