//
// Created by ohmy on 2022-03-08.
//

#ifndef DIRECTTRANSCRIPTION_LGLMS_H
#define DIRECTTRANSCRIPTION_LGLMS_H

#include <casadi/casadi.hpp>

#include <casadi/casadi.hpp>
#include "mropt/Dynamics/Transcription.hpp"
#include "mropt/Dynamics/OdeApprox.hpp"

using namespace casadi;

namespace mropt::Dynamics {
    class LGLms : public Transcription {
    public:
        // For a Legendre polynomial of degree n, and N shooting intervals ones has:
        // X (nx * N)
        // U (nu * N)
        // Tau ( nx * n+1 * N )
        // Let nx be the number of states and nu of controls, e.g. se2 nx = 3 (x,y,o)
//        casadi::MX &X; // State Shooting vars
//        casadi::MX &U; // Control vars
        casadi::MX W; // States + collocation states
        casadi::MX Tau; // Collocation nodes vars
//        casadi::Function f;
//        casadi::Function J;
        Slice all;
//        std::vector<MX> g;


        // Legendre-Gauss-Lobato parameters
        DM tau; // Collocation nodes in [-1, 1]
        DM D;   // Differentiation matrix
        DM w;   // Weight quadrature
        int n;  // Polynomial degree
        MX J_;

        explicit LGLms(const std::shared_ptr<OdeApprox> &ode_approx, int degree)
                : Transcription(ode_approx) {
            this->n = degree;
            create_LGL_params(degree);
        }



        MX integrated_cost(MX t0, MX tf, int N);

        std::vector<MX> get_constraints(casadi::Opti& ocp) override;
        void set_J_real() override;
        MX getStates() { return W; };

        int computeN(int inital_N) override;

    private:
        void create_LGL_params(int degree);
    private:
        std::shared_ptr<Transcription> clone(const std::shared_ptr<OdeApprox> ode_approx) const override {
            return std::make_shared<LGLms>(ode_approx, n);
        }

    };
}


#endif //DIRECTTRANSCRIPTION_LGLMS_H
