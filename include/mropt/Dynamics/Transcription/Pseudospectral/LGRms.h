//
// Created by ohmy on 2022-03-24.
//

#ifndef MROPT_LGRMS_H
#define MROPT_LGRMS_H

#include <casadi/casadi.hpp>

#include "mropt/Dynamics/Transcription.hpp"
#include "mropt/Dynamics/OdeApprox.hpp"

using namespace casadi;

namespace mropt::Dynamics {
    class LGRms : public Transcription {
    public:
        // For a Legendre polynomial of degree n, and N shooting intervals ones has:
        // X (nx * N)
        // U (nu * N)
        // Tau ( nx * n+1 * N )
        // Let nx be the number of states and nu of controls, e.g. se2 nx = 3 (x,y,o)
        casadi::MX W; // States + collocation states
        casadi::MX Tau; // Collocation nodes vars
        Slice all;

        // Legendre-Gauss-Radau parameters
        DM tau; // Collocation nodes in [-1, 1]
        DM D;   // Differentiation matrix
        DM w;   // Weight quadrature
        DM E;   // End State interpolation
        int n;  // Polynomial degree
        MX J_;

        explicit LGRms(const std::shared_ptr<OdeApprox> &ode_approx,
                       const std::shared_ptr<mropt::cost::Cost> &cost, int degree)
                : Transcription(ode_approx, cost) {
            this->n = degree;
            create_LGR_params(degree);
        }


        std::vector<MX> get_constraints(casadi::Opti& ocp) override;
        void set_J_real(casadi::Opti &ocp) override;
        void set_J() override;

        MX getStates() { return W; };

        int computeN(int inital_N) override;

    private:
        void create_LGR_params(int degree);
    private:
        std::shared_ptr<Transcription> clone(
                const std::shared_ptr<OdeApprox> ode_approx,
                const std::shared_ptr<mropt::cost::Cost> &cost) const override {
            return std::make_shared<LGRms>(ode_approx, cost, n);
        }

    };
}


#endif //MROPT_LGRMS_H
