//
// Created by ohmy on 2021-12-20.
//

#ifndef MRRM_RECORDER_H
#define MRRM_RECORDER_H

#include <string>
#include <vector>

namespace mropt::util {
    class Recorder {
    public:
        struct Record {
            int R; // Number of Robots
            double t; // Computation time final solution
            double t_1; // Computation time 1st solution
            double t_setup; // Setup time
            double cost; // Cost of final solution
            double cost_1; // Cost of first solution
            double T; // Trajectorie's time
            double N; // Discritization knots
            double D; // Path Length
            double A; // Scenario Area
            double L; // Robot's Footprint Length
            int fail_status; // Number of Fail type 1 is Success, fail otherwise
            int iter_1;
            Record& dummy_failed_record(){
                this->fail_status = -1;
                return *this;
            }
        };

        void addRecord(std::string filename, Record &record);

        void addRecord(std::string filename, Record &&record);

        void plot_scaling_experiments(std::string file_location);

        void plot_path_length_experiments(std::string file_location);

        void plot_problem_hardness_experiments_41(std::string file_location);

        void plot_problem_hardness_experiments_42(std::string file_location);

        std::vector<Record> read_records(std::string file_name);

    };
}


#endif //MRRM_RECORDER_H
