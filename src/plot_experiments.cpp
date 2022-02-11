//
// Created by ohmy on 2021-12-21.
//

#include "mropt/util/Recorder.h"

int main(){
    mropt::util::Recorder recorder_helper;
        std::string file_name = "/home/ohmy/js_ws/github_repos/mrrm/records";
//     recorder_helper.plot_path_length_experiments(
//             file_name);
    // recorder_helper.plot_scaling_experiments(
    //         file_name);
    recorder_helper.plot_problem_hardness_experiments_42(
            "/home/ohmy/js_ws/github_repos/mrrm/records/Exp4.2-scenario_percentage_fixed/19jan/"
            );
    // recorder_helper.plot_problem_hardness_experiments_41(
    //         "/home/ohmy/js_ws/github_repos/mrrm/records/Exp4.1-scenario_percentage_vary/19jan/"
    // );
    return 0;
}