//
// Created by ohmy on 2021-09-16.
//

#ifndef MROPT_INCLUDE_MROPT_PROBLEM_SHAREDDATA_H
#define MROPT_INCLUDE_MROPT_PROBLEM_SHAREDDATA_H

#include <casadi/casadi.hpp>
#include <mropt/Collisions/Collisions.h>
#include <mropt/Problem/DistributedRobot.h>
#include "mpi.h"

using namespace casadi;
namespace mropt::Problem {
    class DistributedRobot;
    class SharedData {
        int R;
        int N;
        Slice all;
    public:
        explicit SharedData(
                int N,
                int R) : R(R), N(N) {
            rho = new double;
            *rho = rho_0;
            safe_distance = new double[R];
            dynamic_violation = new double[R];
            x = new double[R * (N + 1)];
            y = new double[R * (N + 1)];
            x_p = new double[R * (N + 1)];
            y_p = new double[R * (N + 1)];
            /*z = new double[R * (R - 1) * (N + 1)];
            mu12 = new double[R * (R - 1) * (N + 1)];*/
            z_x = new double[R  * (N + 1)];
            z_y = new double[R  * (N + 1)];
            z_x_p = new double[R  * (N + 1)];
            z_y_p = new double[R  * (N + 1)];
            mu_x = new double[R  * (N + 1)];
            mu_y = new double[R  * (N + 1)];
            size_x = size_y = R * (N + 1);
            size_sd = R;
            /*size_z = size_mu12 = R * (R - 1) * (N + 1);*/
            size_z = size_mu12 = R * (N+1);
            cost_r = std::vector<std::vector<double>>(R, std::vector<double>{});
            time_r = std::vector<std::vector<double>>(R, std::vector<double>{});
            traj_r = std::vector<std::vector<double>>(R, std::vector<double>{});
        }

        virtual ~SharedData() {
            delete rho;
            delete[] safe_distance;
            delete[] dynamic_violation;
            delete[] x;
            delete[] y;
            delete[] x_p;
            delete[] y_p;
            delete[] z_x;
            delete[] z_y;
            delete[] z_x_p;
            delete[] z_y_p;
            delete[] mu_x;
            delete[] mu_y;

        }
        // Actual shared data
        int size_x, size_y, size_z, size_mu12, size_sd;
        double *rho;
        double rho_0{0.1};//10000
        double *safe_distance;
        double *dynamic_violation;
        double *x;
        double *y;
        double *x_p;
        double *y_p;
        double *z_x;
        double *z_y;
        double *z_x_p;
        double *z_y_p;
        double *mu_x;
        double *mu_y;
        double mu12_init{0.0};

        // Plotting vars
        std::vector<double> g12_sum{};
        std::vector<double> g21_sum{};
        std::vector<double> z_sum{};
        std::vector<double> greal_sum{};
        std::vector<double> mu12_sum{};
        std::vector<std::vector<double>> cost_r;
        std::vector<double> cost_mr;
        std::vector<double> min_clearance;
        std::vector<std::vector<double>> time_r;
        std::vector<std::vector<double>> traj_r;
        std::vector<double> traj_r_sum;

        casadi::DM Xcurr(int r, int k);
        casadi::DM Xcurr(int r);
        casadi::DM Xprev(int r, int k);
        casadi::DM Xprev(int r);
/*  double Z_val(int r1, int r2, int k);
  double Mu12_val(int r1, int r2, int k);*/
        casadi::DM Z_val(int r, int k);
        casadi::DM Z_val_p(int r, int k);
        casadi::DM Mu12_val(int r, int k);

        void printX();
        void printY();
        void printZ();
        void printMu12();
        void printXp();
        double getSafetyDistance(int robot_id) const { return safe_distance[robot_id]; }
        void initializeZandMu(const mropt::collisions::Collisions &col, int proc_id);
        void init_sync(std::shared_ptr<DistributedRobot> &robot);
        void sync(std::shared_ptr<DistributedRobot> &robot);
        void update_z_mu12(const mropt::collisions::Collisions &col, int proc_id);

        double dynamicsViolation();
        double sumColViolation(const mropt::collisions::Collisions &col);
        double maxColViolation(const mropt::collisions::Collisions &col);
        double actual_cost(std::shared_ptr<DistributedRobot> &robot);
        void getMRTrajectory(
                std::vector<std::vector<double>> &x,
                std::vector<std::vector<double>> &y,
                std::vector<std::vector<double>> &o,
                std::vector<std::vector<std::vector<double>>> &u,
                const std::shared_ptr<DistributedRobot> &robot);
        void getMRTrajectory(
                std::vector<std::vector<std::vector<double>>> &x,
                std::vector<std::vector<std::vector<double>>> &u,
                const std::shared_ptr<DistributedRobot> &robot);

        void plot(
                std::shared_ptr<DistributedRobot> &robot,
                double solve_time_r);

        double X(int r, int k) const { return x[k + r * (N + 1)]; }
        double Y(int r, int k) const { return y[k + r * (N + 1)]; }
    private:

/*  void setZval(int r1, int r2, int k, double val);
  void setMu12val(int r1, int r2, int k, double val);*/
        void setZval(int r, int k, double x, double y);
        void setZval_p(int r, int k, double x, double y);
        void setMu12val(int r, int k, double x, double y);

        double c(int k);
        int iters;
    };
}

#endif //MROPT_INCLUDE_MROPT_PROBLEM_SHAREDDATA_H
