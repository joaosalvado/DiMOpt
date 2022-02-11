//
// Created by ohmy on 2021-12-20.
//

#include "Recorder.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <matplotlibcpp.h>
#include <matplot/matplot.h>
using namespace mropt::util;
namespace plt = matplotlibcpp;

void Recorder::addRecord(std::string filename, Record &record){
    std::stringstream ss;
    ss << record.R << ","
       << record.A << ","
       << record.D << ","
       << record.L << ","
       << record.N << ","
       << record.T << ","
       << record.cost << ","
       << record.cost_1 << ","
       << record.t << ","
       << record.t_1 << ","
       << record.t_setup << ","
       << record.fail_status << ","
       << record.iter_1;
    std::ofstream log(filename, std::ios_base::app | std::ios_base::out);
    log << ss.str() << std::endl;
}
void Recorder::addRecord(std::string filename, Record &&record){
    std::stringstream ss;
    ss << record.R << ","
       << record.A << ","
       << record.D << ","
       << record.L << ","
       << record.N << ","
       << record.T << ","
       << record.cost << ","
       << record.cost_1 << ","
       << record.t << ","
       << record.t_1 << ","
       << record.t_setup << ","
       << record.fail_status;
    std::ofstream log(filename, std::ios_base::app | std::ios_base::out);
    log << ss.str() << std::endl;
}

template <typename T, typename Compare>
std::vector<std::size_t> sort_permutation(
        const std::vector<T>& vec,
        Compare& compare)
{
    std::vector<std::size_t> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(),
              [&](std::size_t i, std::size_t j){ return compare(vec[i], vec[j]); });
    return p;
}

template <typename T>
std::vector<T> apply_permutation(
        const std::vector<T>& vec,
        const std::vector<std::size_t>& p)
{
    std::vector<T> sorted_vec(vec.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(),
                   [&](std::size_t i){ return vec[i]; });
    return sorted_vec;
}


void Recorder::plot_path_length_experiments(std::string file_location){
    std::stringstream file_distributed;
    std::stringstream file_coupled;
    file_distributed << file_location << "/distributed.txt";
    file_coupled << file_location << "/coupled.txt";

    std::vector<Record> records_distributed = read_records(file_distributed.str());
    std::vector<Record> records_coupled = read_records(file_coupled.str());

    // Coupled
    // std::vector<double> R_coupled;
    // std::vector<double> T_coupled;
    // std::vector<double> t_coupled;
    // std::for_each(records_coupled.begin(), records_coupled.end(),
    //               [&](const Record &record){
    //                   R_coupled.push_back(record.R);
    //                   T_coupled.push_back(record.T);
    //                   t_coupled.push_back(record.t);
    //               });
    // // Distributed
    // std::vector<double> R_distributed;
    // std::vector<double> T_distributed;
    // std::vector<double> t_distributed;
    // std::for_each(records_distributed.begin(), records_distributed.end(),
    //               [&](const Record &record){
    //                   R_distributed.push_back(record.R);
    //                   T_distributed.push_back(record.T);
    //                   t_distributed.push_back(record.t);
    //               });

    std::map<std::string, std::string> c_options;
    c_options["alpha"] = "0.9";
    c_options["color"] = "grey";
    std::map<std::string, std::string> d_options;
    d_options["alpha"] = "0.9";
    d_options["color"] = "green";

    std::map<std::string, std::string> surface_options{
/*            {"rstride","8"},
            {"cstride", "8"},*/
            {"alpha", "0.9"},
            {"antialiased","False"},
            {"linewidth", "0"},
/*            {"extend3d","1"}*/
    };


/*
    plt::title("Path Length");
    plt::scatter(R_coupled, T_coupled, t_coupled,10, c_options,  1);
    plt::scatter(R_distributed, T_distributed, t_distributed,10,  d_options, 1);
    //plt::contour(R_distributed, T_distributed, t_distributed,c_options);
    //plt::legend();
    plt::show();
*/

    std::vector<std::string> labels;
    std::vector<double> x_axis;
    std::map<int, int> key;
    int count = 0;
    std::map<std::string, std::string> options{
            {"sym"," "}
            //{"flierprops", " {{markerfacecolor, r}, {marker,  s}}"}
    };
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      if( x_axis.size() == 0 ){
                          labels.push_back(std::to_string(record.R));
                          x_axis.push_back(record.R);
                          key.insert(std::make_pair(record.R,count++));
                      } else{
                          if(x_axis.back() != record.R){
                              labels.push_back(std::to_string(record.R));
                              x_axis.push_back(record.R);
                              key.insert(std::make_pair(record.R,count++));
                          }
                      }
                  });


    //COUPLED
auto  R_coupled_ = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
auto  T_coupled_ = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
auto  t_coupled_ = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
 std::for_each(records_coupled.begin(), records_coupled.end(),
                  [&](const Record &record){
                      R_coupled_[key[record.R]].push_back(record.R);
                      T_coupled_[key[record.R]].push_back(record.T);
                      t_coupled_[key[record.R]].push_back(std::log(record.t));
                  });

    // // Countour and Surface
    // std::vector<std::vector<double>> R_coupled_;
    // std::vector<std::vector<double>> T_coupled_;
    // std::vector<std::vector<double>> t_coupled_;
    // for(int r = 2; r <= 18; ++r){
    //     std::vector<double> tc1_{};
    //     for(int exp = 0; exp < 200; exp ++ ){
    //         tc1_.push_back(t_coupled[(r-2)*200 + exp]);
    //     }
    //     //quantile 90
    //     auto const Q_90 = tc1_.size() * 0.95;
    //     std::nth_element(tc1_.begin(), tc1_.begin() + Q_90,  tc1_.end());
    //     auto bound90 = tc1_[(int)Q_90];
    //     std::vector<double> rc_{};
    //     std::vector<double> Tc_{};
    //     std::vector<double> tc_{};
    //     for(int exp = 0; exp < 200; exp ++ ){
    //         if(t_coupled[(r - 2) * 200 + exp]<=bound90) {
    //             rc_.push_back(R_coupled[(r - 2) * 200 + exp]);
    //             Tc_.push_back(T_coupled[(r - 2) * 200 + exp]);
    //             tc_.push_back(t_coupled[(r - 2) * 200 + exp]);
    //         }
    //     }

    //     R_coupled_.push_back(rc_);
    //     auto compare = std::less<double>();
    //     auto p = sort_permutation(Tc_, compare);
    //     Tc_ = apply_permutation(Tc_, p);
    //     T_coupled_.push_back(Tc_);
    //     tc_ = apply_permutation(tc_,p);
    //     t_coupled_.push_back(tc_);
    // }
    // plt::plot_surface(R_coupled_, T_coupled_, t_coupled_, surface_options);
    // plt::show();

/*
    auto x = R_coupled_;
    auto y = T_coupled_;
    auto z = t_coupled_;
    matplot::surf(x, y, z);
    //matplot::view(2);
    matplot::show();
*/


    using namespace matplot;
    tiledlayout(2, 2);
    auto x = R_coupled_;
    auto y = T_coupled_;
    auto z = t_coupled_;
    nexttile();
    surf(x, y, z);
    nexttile();
    matplot::surf(x, y, z);
    view(90, 0);
    auto ax = nexttile();
    matplot::surf(x, y, z);
    view(0, 0);
/*    nexttile();
    matplot::surf(x, y, z);
    view(2);
    matplot::colorbar();*/
/*    nexttile();
    matplot::surf(x, y, z);
    view(2);*/
    //auto ax = nexttile(2);
    //colormap(ax, palette::cool());
    show();



    // DISTRIBUTED
    // Countour and Surface
    auto  R_distributed = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
auto  T_distributed = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
auto  t_distributed = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
 std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      R_distributed[key[record.R]].push_back(record.R);
                      T_distributed[key[record.R]].push_back(record.T);
                      t_distributed[key[record.R]].push_back(std::log(record.t));
                  });
        auto  R_distributed_ = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
auto  T_distributed_ = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
auto  t_distributed_ = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
 
    for(int r=0; r < R_distributed.size(); ++r){
        std::vector dummy_t = t_distributed[r];
        auto const Q_90 = dummy_t.size() * 0.90;
        std::nth_element(dummy_t.begin(), dummy_t.begin() + Q_90,  dummy_t.end());
        auto bound90 = dummy_t[(int)Q_90];
        for(int i=0; i < R_distributed[r].size(); ++i){
            if(t_distributed[r][i] <= bound90){
                R_distributed_[key[R_distributed[r][i]]].push_back(R_distributed[r][i]);
                T_distributed_[key[R_distributed[r][i]]].push_back(T_distributed[r][i]);
                t_distributed_[key[R_distributed[r][i]]].push_back(t_distributed[r][i]);
            }
        }
    }
    // std::vector<std::vector<double>> R_distributed_;
    // std::vector<std::vector<double>> T_distributed_;
    // std::vector<std::vector<double>> t_distributed_;
    // for(int r = 2; r <= 18; ++r){
    //     std::vector<double> td1_{};
    //     for(int exp = 0; exp < 200; exp ++ ){
    //         td1_.push_back(t_distributed[(r-2)*200 + exp]);
    //     }
    //     //quantile 90
    //     auto const Q_90 = td1_.size() * 0.90;
    //     std::nth_element(td1_.begin(), td1_.begin() + Q_90,  td1_.end());
    //     auto bound90 = td1_[(int)Q_90];
    //     std::vector<double> rd_{};
    //     std::vector<double> Td_{};
    //     std::vector<double> td_{};
    //     for(int exp = 0; exp < 200; exp ++ ){
    //         if(t_distributed[(r - 2) * 200 + exp] <= bound90) {
    //             rd_.push_back(R_distributed[(r - 2) * 200 + exp]);
    //             Td_.push_back(T_distributed[(r - 2) * 200 + exp]);
    //             td_.push_back(t_distributed[(r - 2) * 200 + exp]);
    //         }
    //     }

    //     R_distributed_.push_back(rd_);
    //     auto compare = std::less<double>();
    //     auto p = sort_permutation(Td_, compare);
    //     Td_ = apply_permutation(Td_, p);
    //     T_distributed_.push_back(Td_);
    //     td_ = apply_permutation(td_,p);
    //     t_distributed_.push_back(td_);
    // }
    // plt::plot_surface(R_distributed_, T_distributed_, t_distributed_);
    // plt::show();


    using namespace matplot;
    matplot::figure();
    tiledlayout(2, 2);
    auto x_d = R_distributed_;
    auto y_d = T_distributed_;
    auto z_d = t_distributed_;
    nexttile();
    surf(x_d, y_d, z_d);
    nexttile();
    surf(x_d, y_d, z_d);
    view(90, 0);
    nexttile();
    surf(x_d, y_d, z_d);
    view(0, 0);
    show();


    // On the same figure
    matplot::figure();
    tiledlayout(1, 2);

    nexttile();
    surf(x_d, y_d, z_d);
    view(2);
    matplot::colorbar();

    nexttile();
    matplot::surf(x, y, z);
    view(2);
    matplot::colorbar();
    show();


    std::string filename = "/home/ohmy/js_ws/github_repos/mrrm/records/c3d.data";
    for( int i = 0;  i < x.size(); i++) {
        for(int j = 0; j < x.size(); j++) {
            std::stringstream ss;
            ss << x[i][j] << ","
               << y[i][j] << ","
               << z[i][j];
            std::ofstream log(filename, std::ios_base::app | std::ios_base::out);
            log << ss.str() << std::endl;
        }
    }

    filename = "/home/ohmy/js_ws/github_repos/mrrm/records/d3d.data";
    for( int i = 0;  i < x_d.size(); i++) {
        for(int j = 0; j < x_d.size(); j++) {
            std::stringstream ss;
            ss << x_d[i][j] << ","
               << y_d[i][j] << ","
               << z_d[i][j];
            std::ofstream log(filename, std::ios_base::app | std::ios_base::out);
            log << ss.str() << std::endl;
        }
    }




}

void Recorder::plot_scaling_experiments(std::string file_location){
    std::stringstream file_distributed;
    std::stringstream file_coupled;
    file_distributed << file_location << "/distributed.txt";
    file_coupled << file_location << "/coupled.txt";

    std::vector<Record> records_distributed = read_records(file_distributed.str());
    std::vector<Record> records_coupled = read_records(file_coupled.str());

    //Read from file
    // 0.1) First Iter Number
/*    std::vector<int> iter_1;
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      while(iter_1.size() < record.R){
                          iter_1.push_back(1);
                      }
                      iter_1[record.R-1] = record.
                  });
    */
    std::vector<std::string> labels;
    std::vector<double> x_axis;
    std::map<int, int> key;
    int count = 0;
    std::map<std::string, std::string> options{
            {"sym"," "}
            //{"flierprops", " {{markerfacecolor, r}, {marker,  s}}"}
    };
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      if( x_axis.size() == 0 ){
                          labels.push_back(std::to_string(record.R));
                          x_axis.push_back(record.R);
                          key.insert(std::make_pair(record.R,count++));
                      } else{
                          if(x_axis.back() != record.R){
                              labels.push_back(std::to_string(record.R));
                              x_axis.push_back(record.R);
                              key.insert(std::make_pair(record.R,count++));
                          }
                      }
                  });

    // 0.2) failed coupled
    auto failed_coupled  = std::vector<std::pair<int,int>>(x_axis.size(), std::pair<int,int>());
    std::for_each(records_coupled.begin(), records_coupled.end(),
                  [&](const Record &record){
                      if(record.fail_status == 1){
                          failed_coupled[key[record.R]].first++;
                      } else{
                          failed_coupled[key[record.R]].second++;
                      }
                  });
    //0.3) failed distributed
    auto failed_distributed = std::vector<std::pair<int,int>>(x_axis.size(), std::pair<int,int>());
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      if(record.fail_status == 1){
                          failed_distributed[key[record.R]].first++;
                      } else{
                          failed_distributed[key[record.R]].second++;
                      }
                  });
    // 1.1) t coupled - gather
    auto t_final_coupled =  std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());
    std::for_each(records_coupled.begin(), records_coupled.end(),
                  [&t_final_coupled, &key](const Record &record){
                      t_final_coupled[key[record.R]].push_back(record.t);
                  });
    // 1.2) t coupled - quantiles
    std::vector<double> t_final_coupled_q10;
    std::vector<double> t_final_coupled_q50;
    std::vector<double> t_final_coupled_q90;
    for (int r = 0; r < t_final_coupled.size(); ++r){
        auto const Q_10 = t_final_coupled[r].size() * 0.1;
        auto const Q_50 = t_final_coupled[r].size() * 0.5;
        auto const Q_90 = t_final_coupled[r].size() * 0.9;
        std::nth_element(t_final_coupled[r].begin(), t_final_coupled[r].begin() + Q_10, t_final_coupled[r].end());
        t_final_coupled_q10.push_back(t_final_coupled[r][(int)Q_10]);
        std::nth_element(t_final_coupled[r].begin(), t_final_coupled[r].begin() + Q_50, t_final_coupled[r].end());
        t_final_coupled_q50.push_back(t_final_coupled[r][(int)Q_50]);
        std::nth_element(t_final_coupled[r].begin(), t_final_coupled[r].begin() + Q_90,  t_final_coupled[r].end());
        t_final_coupled_q90.push_back(t_final_coupled[r][(int)Q_90]);

    }
    // 2.1) t distributed - gather
    auto t_final_distributed = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());;
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&t_final_distributed, &key](const Record &record){
                      t_final_distributed[key[record.R]].push_back(record.t);
                  });
    // 2.2) t distributed - quantiles
    std::vector<double> t_final_distributed_q10;
    std::vector<double> t_final_distributed_q50;
    std::vector<double> t_final_distributed_q90;
    for (int r = 0; r < t_final_distributed.size(); ++r){
        auto const Q_10 = t_final_distributed[r].size() * 0.1;
        auto const Q_50 = t_final_distributed[r].size() * 0.5;
        auto const Q_90 = t_final_distributed[r].size() * 0.85;
        std::nth_element(t_final_distributed[r].begin(), t_final_distributed[r].begin() + Q_10, t_final_distributed[r].end());
        t_final_distributed_q10.push_back(t_final_distributed[r][(int)Q_10]);
        std::nth_element(t_final_distributed[r].begin(), t_final_distributed[r].begin() + Q_50, t_final_distributed[r].end());
        t_final_distributed_q50.push_back(t_final_distributed[r][(int)Q_50]);
        std::nth_element(t_final_distributed[r].begin(), t_final_distributed[r].begin() + Q_90,  t_final_distributed[r].end());
        t_final_distributed_q90.push_back(t_final_distributed[r][(int)Q_90]);

    }
    // 3.1) t 1 - gather
    auto t_1_distributed = std::vector<std::vector<double>>(x_axis.size(), std::vector<double>());;
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&t_1_distributed, &key](const Record &record){
                      t_1_distributed[key[record.R]].push_back(record.t_1);
                  });
    // 3.2) t 1 - quantiles
    std::vector<double> t_1_distributed_q10;
    std::vector<double> t_1_distributed_q50;
    std::vector<double> t_1_distributed_q90;
    for (int r = 0; r < t_1_distributed.size(); ++r){
        auto const Q_10 = t_1_distributed[r].size() * 0.1;
        auto const Q_50 = t_1_distributed[r].size() * 0.5;
        auto const Q_90 = t_1_distributed[r].size() * 0.9;
        std::nth_element(t_1_distributed[r].begin(), t_1_distributed[r].begin() + Q_10, t_1_distributed[r].end());
        t_1_distributed_q10.push_back(t_1_distributed[r][(int)Q_10]);
        std::nth_element(t_1_distributed[r].begin(), t_1_distributed[r].begin() + Q_50, t_1_distributed[r].end());
        t_1_distributed_q50.push_back(t_1_distributed[r][(int)Q_50]);
        std::nth_element(t_1_distributed[r].begin(), t_1_distributed[r].begin() + Q_90,  t_1_distributed[r].end());
        t_1_distributed_q90.push_back(t_1_distributed[r][(int)Q_90]);
    }
    // 4.1) Cost coupled - gather
    std::vector<std::vector<double>> cost_coupled;
    std::for_each(records_coupled.begin(), records_coupled.end(),
                  [&cost_coupled](const Record &record){
                      while(cost_coupled.size() < record.R){
                          cost_coupled.push_back(std::vector<double>{});
                      }
                      cost_coupled[record.R-1].push_back(record.cost/record.R);
                  });
    // 4.2) Cost coupled - quantiles
    std::vector<double> cost_coupled_q10;
    std::vector<double> cost_coupled_q50;
    std::vector<double> cost_coupled_q90;
    for (int r = 1; r < cost_coupled.size(); ++r){
        auto const Q_10 = cost_coupled[r].size() * 0.1;
        auto const Q_50 = cost_coupled[r].size() * 0.5;
        auto const Q_90 = cost_coupled[r].size() * 0.9;
        std::nth_element(cost_coupled[r].begin(), cost_coupled[r].begin() + Q_10, cost_coupled[r].end());
        cost_coupled_q10.push_back(cost_coupled[r][(int)Q_10]);
        std::nth_element(cost_coupled[r].begin(), cost_coupled[r].begin() + Q_50, cost_coupled[r].end());
        cost_coupled_q50.push_back(cost_coupled[r][(int)Q_50]);
        std::nth_element(cost_coupled[r].begin(), cost_coupled[r].begin() + Q_90,  cost_coupled[r].end());
        cost_coupled_q90.push_back(cost_coupled[r][(int)Q_90]);
    }
    // 5.1) Cost distributed - gather
    std::vector<std::vector<double>> cost_distributed;
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&cost_distributed](const Record &record){
                      while(cost_distributed.size() < record.R){
                          cost_distributed.push_back(std::vector<double>{});
                      }
                      cost_distributed[record.R-1].push_back(record.cost/record.R);
                  });
    // 5.2) Cost Distributed - quantiles
    std::vector<double> cost_distributed_q10;
    std::vector<double> cost_distributed_q50;
    std::vector<double> cost_distributed_q90;
    for (int r = 1; r < cost_distributed.size(); ++r){
        auto const Q_10 = cost_distributed[r].size() * 0.1;
        auto const Q_50 = cost_distributed[r].size() * 0.5;
        auto const Q_90 = cost_distributed[r].size() * 0.9;
        std::nth_element(cost_distributed[r].begin(), cost_distributed[r].begin() + Q_10, cost_distributed[r].end());
        cost_distributed_q10.push_back(cost_distributed[r][(int)Q_10]);
        std::nth_element(cost_distributed[r].begin(), cost_distributed[r].begin() + Q_50, cost_distributed[r].end());
        cost_distributed_q50.push_back(cost_distributed[r][(int)Q_50]);
        std::nth_element(cost_distributed[r].begin(), cost_distributed[r].begin() + Q_90,  cost_distributed[r].end());
        cost_distributed_q90.push_back(cost_distributed[r][(int)Q_90]);
    }
    // 6.1 ) Cost distributed 1 - gather
    std::vector<std::vector<double>> cost_1_distributed;
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&cost_1_distributed](const Record &record){
                      while(cost_1_distributed.size() < record.R){
                          cost_1_distributed.push_back(std::vector<double>{});
                      }
                      cost_1_distributed[record.R-1].push_back(record.cost_1/record.R);
                  });
    // 6.2) COst distributed 1 - quantiles
    std::vector<double> cost_distributed1_q10;
    std::vector<double> cost_distributed1_q50;
    std::vector<double> cost_distributed1_q90;
    for (int r = 1; r < cost_distributed.size(); ++r){
        auto const Q_10 = cost_1_distributed[r].size() * 0.1;
        auto const Q_50 = cost_1_distributed[r].size() * 0.5;
        auto const Q_90 = cost_1_distributed[r].size() * 0.9;
        std::nth_element(cost_1_distributed[r].begin(), cost_1_distributed[r].begin() + Q_10, cost_1_distributed[r].end());
        cost_distributed1_q10.push_back(cost_1_distributed[r][(int)Q_10]);
        std::nth_element(cost_1_distributed[r].begin(), cost_1_distributed[r].begin() + Q_50, cost_1_distributed[r].end());
        cost_distributed1_q50.push_back(cost_1_distributed[r][(int)Q_50]);
        std::nth_element(cost_1_distributed[r].begin(), cost_1_distributed[r].begin() + Q_90,  cost_1_distributed[r].end());
        cost_distributed1_q90.push_back(cost_1_distributed[r][(int)Q_90]);
    }

    // std::vector<std::string> labels;
    // std::vector<double> x_axis;
    // std::map<std::string, std::string> options{
    //         {"sym"," "}
    //         //{"flierprops", " {{markerfacecolor, r}, {marker,  s}}"}
    // };
    // for(int r = 2; r < t_final_distributed.size()+1; ++r){
    //     labels.push_back(std::to_string(r));
    //     x_axis.push_back(r);
    // }



    std::map<std::string, std::string> coupled_fill_in_between_options =
    {
        {"alpha", "0.2"},
        {"color", "grey"}
       // {"label", "C 10-90%"}
    };
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> coupled_median_options{
            {"color", "grey"},
            {"label", "C 50%"}
    };
    std::map<std::string, std::string> coupled_90_options =
            {
                    {"color" , "grey"},
                    {"linestyle", "dashed"},
                    {"label", "C 90%"}
            };
    std::map<std::string, std::string> coupled_10_options =
            {
                    {"color" , "grey"},
                    {"linestyle", "dashdot"},
                    {"label", "C 10%"}
            };

    std::map<std::string, std::string> distributed_fill_in_between_options;
    distributed_fill_in_between_options["alpha"] = "0.2";
    distributed_fill_in_between_options["color"] = "red";
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> distributed_median_options{
            {"color", "red"},
            {"label", "D 50%"}
    };
    std::map<std::string, std::string> distributed_90_options =
            {
                    {"color" , "red"},
                    {"linestyle", "dashed"},
                    {"label", "D 90%"}
            };
    std::map<std::string, std::string> distributed_10_options =
            {
                    {"color" , "red"},
                    {"linestyle", "dashdot"},
                    {"label", "D 10%"}
            };

    std::map<std::string, std::string> distributed1_fill_in_between_options;
    distributed1_fill_in_between_options["alpha"] = "0.2";
    distributed1_fill_in_between_options["color"] = "orange";
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> distributed1_median_options{
            {"color","orange"},
            {"label", "D1 50%"}
    };
    std::map<std::string, std::string> distributed1_90_options =
            {
                    {"color" , "orange"},
                    {"linestyle", "dashed"},
                    {"label", "D1 90%"}
            };
    std::map<std::string, std::string> distributed1_10_options =
            {
                    {"color" , "orange"},
                    {"linestyle", "dashdot"},
                    {"label", "D1 10%"}
            };



/*    // 6X6
    plt::subplot(2,3,1);
    plt::title("Coupled");
    plt::boxplot(t_final_coupled,labels, options);
    plt::fill_between(x_axis, t_final_coupled_q10, t_final_coupled_q90, coupled_fill_in_between_options);
    plt::plot(x_axis,t_final_coupled_q50, coupled_median_options);
    plt::legend();


    plt::subplot(2,3,2);
    plt::title("Distributed");
    plt::boxplot(t_final_distributed, labels, options);
    plt::fill_between(x_axis, t_final_distributed_q10, t_final_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_axis,t_final_distributed_q50, distributed_median_options);
    plt::legend();


    plt::subplot(2,3,3);
    plt::title("Distributed 1");
    plt::boxplot(t_1_distributed, labels, options);
    plt::fill_between(x_axis, t_1_distributed_q10, t_1_distributed_q90, distributed1_fill_in_between_options);
    plt::plot(x_axis,t_1_distributed_q50, distributed1_median_options);
    plt::legend();

    plt::subplot(2,3,4);
    plt::title("Coupled Cost");
    plt::boxplot(cost_coupled,labels, options);
    plt::fill_between(x_axis, cost_coupled_q10, cost_coupled_q90, coupled_fill_in_between_options);
    plt::plot(x_axis,cost_coupled_q50, coupled_median_options);
    plt::legend();

    plt::subplot(2,3,5);
    plt::title("Distributed Cost");
    plt::boxplot(cost_distributed, labels, options);
    plt::fill_between(x_axis, cost_distributed_q10, cost_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_axis,cost_distributed_q50, distributed_median_options);
    plt::legend();

    plt::subplot(2,3,6);
    plt::title("Distributed 1 Cost");
    plt::boxplot(cost_1_distributed, labels, options);
    plt::fill_between(x_axis, cost_distributed1_q10, cost_distributed1_q90, distributed1_fill_in_between_options);
    plt::plot(x_axis,cost_distributed1_q50, distributed1_median_options);
    plt::legend();
    plt::show();

    // Time together plot
    plt::fill_between(x_axis, t_final_coupled_q10, t_final_coupled_q90, coupled_fill_in_between_options);
    plt::plot(x_axis,t_final_coupled_q50, coupled_median_options);

    plt::fill_between(x_axis, t_final_distributed_q10, t_final_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_axis,t_final_distributed_q50, distributed_median_options);

    plt::fill_between(x_axis, t_1_distributed_q10, t_1_distributed_q90, distributed1_fill_in_between_options);
    plt::plot(x_axis,t_1_distributed_q50, distributed1_median_options);
    plt::legend();
    plt::show();*/

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<double> x_;
    for(int i = 1; i < x_axis.size()+1; ++i){
        x_.push_back(i);
    }
    // Time in rows
    plt::subplot(3,1,1);
    plt::title("Coupled");
    plt::boxplot(t_final_coupled,labels, options);
    plt::fill_between(x_, t_final_coupled_q10, t_final_coupled_q90, coupled_fill_in_between_options);
    plt::plot(x_,t_final_coupled_q50, coupled_median_options);
    plt::ylim(0,605);
    //plt::legend();


    plt::subplot(3,1,2);
    plt::title("Distributed");
    plt::boxplot(t_final_distributed, labels, options);
    plt::fill_between(x_, t_final_distributed_q10, t_final_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_,t_final_distributed_q50, distributed_median_options);
    plt::ylim(0, 13);
   //plt::legend();


    plt::subplot(3,1,3);
    plt::title("Distributed 1");
    plt::boxplot(t_1_distributed, labels, options);
    plt::fill_between(x_, t_1_distributed_q10, t_1_distributed_q90, distributed1_fill_in_between_options);
    plt::plot(x_,t_1_distributed_q50, distributed1_median_options);
    plt::ylim(0.0, 4.5);
    //plt::legend();
    plt::show();

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    std::size_t const half_size = t_final_coupled.size() / 2;

    std::vector<double> x_1(x_.begin(), x_.begin() + half_size);
    std::vector<double> x_2 = x_1; x_2.push_back(x_1.size()+1);
   
    std::vector<double> x_axis_1(x_axis.begin(), x_axis.begin() + half_size);
    std::vector<double> x_axis_2(x_axis.begin() + half_size, x_axis.end() );
    std::vector<double> x_axis_22(x_axis.begin()+ half_size,  x_axis.end() );

    std::vector<std::string> labels_1(labels.begin(), labels.begin() + half_size);
    std::vector<std::string> labels_2(labels.begin() + half_size, labels.end() );
    std::vector<std::string> labels_22(labels.begin() + half_size, labels.end());
    
    std::vector<std::vector<double>> t_final_coupled_1(t_final_coupled.begin(), t_final_coupled.begin() + half_size);
    std::vector<std::vector<double>> t_final_coupled_2(t_final_coupled.begin() + half_size, t_final_coupled.end() );
    std::vector<double> t_final_coupled_q10_1(t_final_coupled_q10.begin(), t_final_coupled_q10.begin() + half_size);
    std::vector<double> t_final_coupled_q10_2(t_final_coupled_q10.begin() + half_size, t_final_coupled_q10.end() );
    std::vector<double> t_final_coupled_q50_1(t_final_coupled_q50.begin(), t_final_coupled_q50.begin() + half_size);
    std::vector<double> t_final_coupled_q50_2(t_final_coupled_q50.begin() + half_size, t_final_coupled_q50.end() );
    std::vector<double> t_final_coupled_q90_1(t_final_coupled_q90.begin(), t_final_coupled_q90.begin() + half_size);
    std::vector<double> t_final_coupled_q90_2(t_final_coupled_q90.begin() + half_size, t_final_coupled_q90.end() );

    std::vector<std::vector<double>> t_final_distributed_1(t_final_distributed.begin(), t_final_distributed.begin() + half_size);
    std::vector<std::vector<double>> t_final_distributed_2(t_final_distributed.begin() + half_size, t_final_distributed.end() );
    std::vector<double> t_final_distributed_q10_1(t_final_distributed_q10.begin(), t_final_distributed_q10.begin() + half_size);
    std::vector<double> t_final_distributed_q10_2(t_final_distributed_q10.begin() + half_size, t_final_distributed_q10.end() );
    std::vector<double> t_final_distributed_q50_1(t_final_distributed_q50.begin(), t_final_distributed_q50.begin() + half_size);
    std::vector<double> t_final_distributed_q50_2(t_final_distributed_q50.begin() + half_size, t_final_distributed_q50.end() );
    std::vector<double> t_final_distributed_q90_1(t_final_distributed_q90.begin(), t_final_distributed_q90.begin() + half_size);
    std::vector<double> t_final_distributed_q90_2(t_final_distributed_q90.begin() + half_size, t_final_distributed_q90.end() );

    std::vector<std::vector<double>> t_1_distributed_1(t_1_distributed.begin(), t_1_distributed.begin() + half_size);
    std::vector<std::vector<double>> t_1_distributed_2(t_1_distributed.begin() + half_size, t_1_distributed.end() );
    std::vector<double> t_1_distributed_q10_1(t_1_distributed_q10.begin(), t_1_distributed_q10.begin() + half_size);
    std::vector<double> t_1_distributed_q10_2(t_1_distributed_q10.begin() + half_size, t_1_distributed_q10.end() );
    std::vector<double> t_1_distributed_q50_1(t_1_distributed_q50.begin(), t_1_distributed_q50.begin() + half_size);
    std::vector<double> t_1_distributed_q50_2(t_1_distributed_q50.begin() + half_size, t_1_distributed_q50.end() );
    std::vector<double> t_1_distributed_q90_1(t_1_distributed_q90.begin(), t_1_distributed_q90.begin() + half_size);
    std::vector<double> t_1_distributed_q90_2(t_1_distributed_q90.begin() + half_size, t_1_distributed_q90.end() );

    // Time in rows
    plt::subplot(3,2,1);
    // plt::title("Coupled");
    plt::boxplot(t_final_coupled_1,labels_1, options);
    plt::fill_between(x_1, t_final_coupled_q10_1, t_final_coupled_q90_1, coupled_fill_in_between_options);
    plt::plot(x_1,t_final_coupled_q50_1, coupled_median_options);
    plt::ylim(0,19);

    plt::subplot(3,2,2);
    // plt::title("Coupled");
    plt::boxplot(t_final_coupled_2,labels_2, options);
    plt::fill_between(x_2, t_final_coupled_q10_2, t_final_coupled_q90_2, coupled_fill_in_between_options);
    plt::plot(x_2,t_final_coupled_q50_2, coupled_median_options);
    //plt::ylim(0,605);
    //plt::legend();


    plt::subplot(3,2,3);
    // plt::title("Distributed");
    plt::boxplot(t_final_distributed_1, labels_1, options);
    plt::fill_between(x_1, t_final_distributed_q10_1, t_final_distributed_q90_1, distributed_fill_in_between_options);
    plt::plot(x_1,t_final_distributed_q50_1, distributed_median_options);
    plt::ylim(0, 4);

    plt::subplot(3,2,4);
    // plt::title("Distributed");
    plt::boxplot(t_final_distributed_2, labels_2, options);
    plt::fill_between(x_2, t_final_distributed_q10_2, t_final_distributed_q90_2, distributed_fill_in_between_options);
    plt::plot(x_2,t_final_distributed_q50_2, distributed_median_options);
    plt::ylim(0, 13);
   //plt::legend();


    plt::subplot(3,2,5);
    // plt::title("Distributed 1");
    plt::boxplot(t_1_distributed_1, labels_1, options);
    plt::fill_between(x_1, t_1_distributed_q10_1, t_1_distributed_q90_1, distributed1_fill_in_between_options);
    plt::plot(x_1,t_1_distributed_q50_1, distributed1_median_options);
    plt::ylim(0.0, 1.7);

    plt::subplot(3,2,6);
    // plt::title("Distributed 1");
    plt::boxplot(t_1_distributed_2, labels_2, options);
    plt::fill_between(x_2, t_1_distributed_q10_2, t_1_distributed_q90_2, distributed1_fill_in_between_options);
    plt::plot(x_2,t_1_distributed_q50_2, distributed1_median_options);
    plt::ylim(0.0, 4.5);
    //plt::legend();
    plt::show();


    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////

/*    // COst Plot
    plt::title("Multi-robot Cost");
    //plt::fill_between(x_axis, cost_coupled_q50, cost_coupled_q90, coupled_fill_in_between_options);
    plt::plot(x_axis, cost_coupled_q90, coupled_90_options);
    plt::plot(x_axis,cost_coupled_q50, coupled_median_options);
    plt::plot(x_axis,cost_coupled_q10, coupled_10_options);

    //plt::fill_between(x_axis, cost_distributed_q50, cost_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_axis,cost_distributed_q90, distributed_90_options);
    plt::plot(x_axis,cost_distributed_q50, distributed_median_options);
    plt::plot(x_axis,cost_distributed_q10, distributed_10_options);

    //plt::fill_between(x_axis, cost_distributed1_q50, cost_distributed1_q90, distributed1_fill_in_between_options);
    plt::plot(x_axis,cost_distributed1_q90, distributed1_90_options);
    plt::plot(x_axis,cost_distributed1_q50, distributed1_median_options);
    plt::plot(x_axis,cost_distributed1_q10, distributed1_10_options);

    plt::grid(true);
    plt::legend();
    plt::show();*/

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////


    // Cost Plot
    plt::title("Multi-robot Cost");

    plt::subplot(1,2,1);
    plt::plot(x_axis,cost_distributed_q50, distributed_median_options);
    plt::plot(x_axis,cost_coupled_q50, coupled_median_options);
    plt::plot(x_axis,cost_distributed1_q50, distributed1_median_options);
    plt::grid(true);
    plt::legend();

    plt::subplot(1,2,2);
    plt::plot(x_axis,cost_distributed1_q90, distributed1_90_options);
    plt::plot(x_axis,cost_distributed1_q10, distributed1_10_options);
    plt::plot(x_axis,cost_distributed_q90, distributed_90_options);
    plt::plot(x_axis,cost_distributed_q10, distributed_10_options);
    plt::plot(x_axis, cost_coupled_q90, coupled_90_options);
    plt::plot(x_axis,cost_coupled_q10, coupled_10_options);

    plt::grid(true);
    plt::legend();

    plt::show();



    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////


}



void Recorder::plot_problem_hardness_experiments_41(std::string file_location) {
    std::stringstream file_distributed;
    std::stringstream file_coupled;
    file_distributed << file_location << "/distributed.txt";

    std::vector<Record> records_distributed = read_records(file_distributed.str());
    const double PI = std::atan(1)*4;
    // 0) Get all scenario percentages
    std::vector<std::string> labels;
    std::vector<double> x_axis;
    std::map<double, int> key;
    int count = 0;
    std::map<std::string, std::string> options{
            {"sym"," "}
            //{"flierprops", " {{markerfacecolor, r}, {marker,  s}}"}
    };
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                       // int occupied_area = std::round(100*record.R*PI*record.L*record.L/25);
                      double occupied_area = record.L*record.L*record.R*4*4;
                      if( x_axis.size() == 0 ){
                          labels.push_back(std::to_string(occupied_area));
                          x_axis.push_back(occupied_area);
                          key.insert(std::make_pair(occupied_area,count++));
                      } else{
                          if(x_axis.back() != occupied_area){
                              labels.push_back(std::to_string(occupied_area));
                              x_axis.push_back(occupied_area);
                              key.insert(std::make_pair(occupied_area,count++));
                          }
                      }
                  });

    // 0.1) First Iter Number
    auto iter_1  = std::vector<std::vector<int>>(key.size(), std::vector<int>());
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      //int occupied_area =  std::round(100*record.R*PI*record.L*record.L/25);
                      double occupied_area = record.L*record.L*record.R*4*4;
                      if(record.fail_status == 1){
                      iter_1[ key[occupied_area] ].push_back(record.iter_1);
                      }
                  });
    // 0.2) t distributed - quantiles
    std::vector<double> iter1_q10;
    std::vector<double> iter1_q50;
    std::vector<double> iter1_q90;
    for ( const auto &x : x_axis){
        auto const Q_10 = iter_1[ key[x] ].size() * 0.2;
        auto const Q_50 = iter_1[ key[x] ].size() * 0.5;
        auto const Q_90 = iter_1[ key[x] ].size() * 0.8;
        std::nth_element(iter_1[ key[x] ].begin(), iter_1[ key[x] ].begin() + (int)Q_10, iter_1[ key[x] ].end());
        iter1_q10.push_back(iter_1[ key[x] ][(int)Q_10]);
        std::nth_element(iter_1[ key[x] ].begin(), iter_1[ key[x] ].begin() + (int)Q_50, iter_1[ key[x] ].end());
        iter1_q50.push_back(iter_1[ key[x] ][(int)Q_50]);
        std::nth_element(iter_1[ key[x] ].begin(), iter_1[ key[x] ].begin() + (int)Q_90,  iter_1[ key[x] ].end());
        iter1_q90.push_back(iter_1[ key[x] ][(int)Q_90]);

    }

    // 0.1) failed distributed
    auto failed_count_2 = std::vector<int>(key.size(), 0);
    auto failed_count_3 = std::vector<int>(key.size(), 0);
    auto success_count = std::vector<int>(key.size(), 0);
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      //int occupied_area =  std::round(100*record.R*PI*record.L*record.L/25);
                      double occupied_area = record.L*record.L*record.R*4*4;
                      switch(record.fail_status){
                          case 1:
                              success_count[ key[occupied_area] ]++;
                              break;
                          case -2:
                              failed_count_2[ key[occupied_area] ]++;
                              break;
                          case -3:
                              failed_count_3[ key[occupied_area] ]++;
                              break;
                          default:
                              failed_count_3[ key[occupied_area] ]++;
                      }
                  });
    auto failed_percentage_2 = std::vector<double>(failed_count_2.size(), 0.0);
    for(auto r_id = 0; r_id < failed_count_2.size(); r_id++){
        failed_percentage_2[r_id]
        = (double)failed_count_2[r_id]*100/(failed_count_2[r_id] + failed_count_3[r_id] + success_count[r_id]);
    }
    auto failed_percentage_3 = std::vector<double>(failed_count_3.size(), 0.0);
    for(auto r_id = 0; r_id < failed_count_3.size(); r_id++){
        failed_percentage_3[r_id]
        = (double)failed_count_3[r_id]*100/(failed_count_3[r_id] +failed_count_2[r_id]+ success_count[r_id]);
    }
    auto success_percentage= std::vector<double>(success_count.size(), 0.0);
    for(auto r_id = 0; r_id < success_count.size(); r_id++){
        success_percentage[r_id]
        = (double)success_count[r_id]*100/(failed_count_2[r_id]+ failed_count_3[r_id] + success_count[r_id]);
    }

    // 1.1) t distributed - gather
    auto t_final_distributed = std::vector<std::vector<double>>( key.size(), std::vector<double>());
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&t_final_distributed, &PI, &key](const Record &record){
                      //int occupied_area =  std::round(100*record.R*PI*record.L*record.L/25);
                      double occupied_area = record.L*record.L*record.R*4*4;
                      if(record.fail_status == 1){
                          t_final_distributed[ key[occupied_area] ].push_back(record.t);
                      }
                  });
    // 1.2) t distributed - quantiles
    std::vector<double> t_final_distributed_q10;
    std::vector<double> t_final_distributed_q50;
    std::vector<double> t_final_distributed_q90;
    for ( const auto &x : x_axis){
        auto const Q_10 = t_final_distributed[ key[x] ].size() * 0.2;
        auto const Q_50 = t_final_distributed[ key[x] ].size() * 0.5;
        auto const Q_90 = t_final_distributed[ key[x] ].size() * 0.80;
        std::nth_element(t_final_distributed[ key[x] ].begin(), t_final_distributed[ key[x] ].begin() + (int)Q_10, t_final_distributed[ key[x] ].end());
        t_final_distributed_q10.push_back(t_final_distributed[ key[x] ][(int)Q_10]);
        std::nth_element(t_final_distributed[ key[x] ].begin(), t_final_distributed[ key[x] ].begin() + (int)Q_50, t_final_distributed[ key[x] ].end());
        t_final_distributed_q50.push_back(t_final_distributed[ key[x] ][(int)Q_50]);
        std::nth_element(t_final_distributed[ key[x] ].begin(), t_final_distributed[ key[x] ].begin() + (int)Q_90,  t_final_distributed[ key[x] ].end());
        t_final_distributed_q90.push_back(t_final_distributed[ key[x] ][(int)Q_90]);

    }


    std::map<std::string, std::string> iter1_fill_in_between_options;
    iter1_fill_in_between_options["alpha"] = "0.2";
    iter1_fill_in_between_options["color"] = "grey";
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> iter1_median_options{
            {"color", "grey"},
            {"label", "D 50%"}
    };
    std::map<std::string, std::string> iter1_90_options =
            {
                    {"color" , "grey"},
                    {"linestyle", "dashed"},
                    {"label", "D 90%"}
            };
    std::map<std::string, std::string> iter1_10_options =
            {
                    {"color" , "grey"},
                    {"linestyle", "dashdot"},
                    {"label", "D 10%"}
            };

    std::map<std::string, std::string> distributed_fill_in_between_options;
    distributed_fill_in_between_options["alpha"] = "0.2";
    distributed_fill_in_between_options["color"] = "red";
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> distributed_median_options{
            {"color", "red"},
            {"label", "D 50%"}
    };
    std::map<std::string, std::string> distributed_90_options =
            {
                    {"color" , "red"},
                    {"linestyle", "dashed"},
                    {"label", "D 90%"}
            };
    std::map<std::string, std::string> distributed_10_options =
            {
                    {"color" , "red"},
                    {"linestyle", "dashdot"},
                    {"label", "D 10%"}
            };

    std::map<std::string, std::string> options_bar{
            //{"alpha","0.7"},
            {"color", "orange"}
    };

    std::map<std::string, std::string> success_percentage_options{
            {"color", "blue"},
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // Time in rows
    plt::subplot(3,1,1);
    //plt::title("Distributed");
    //plt::boxplot(t_final_distributed, labels, options);
    plt::fill_between(x_axis, t_final_distributed_q10, t_final_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_axis,t_final_distributed_q50, distributed_median_options);
    //plt::ylim(0, 20);

 /*   plt::subplot(4,1,2);
    plt::title("% Failed");
    plt::bar(failed_percentage_2,"black","-",1.25, options_bar);*/

    plt::subplot(3,1,2);
    plt::fill_between(x_axis,  iter1_q10, iter1_q90, iter1_fill_in_between_options);
    plt::plot(x_axis,iter1_q50, iter1_median_options);

   plt::subplot(3,1,3);
    plt::plot(x_axis,success_percentage, success_percentage_options);
    plt::ylim(0,101);
   plt::show();
    
    // std::vector<std::vector<double>> failed_success = {
    //         success_percentage, failed_percentage_2};
    // matplot::barstacked(failed_success);
    // matplot::show();

}





void Recorder::plot_problem_hardness_experiments_42(std::string file_location) {
    std::stringstream file_distributed;
    std::stringstream file_coupled;
    file_distributed << file_location << "/distributed.txt";

    std::vector<Record> records_distributed = read_records(file_distributed.str());
    const double PI = std::atan(1)*4;
    // 0) Get all scenario percentages
    std::vector<std::string> labels;
    std::vector<double> x_axis;
    std::map<int, int> key;
    int count = 0;
    std::map<std::string, std::string> options{
            {"sym"," "}
            //{"flierprops", " {{markerfacecolor, r}, {marker,  s}}"}
    };
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      if( x_axis.size() == 0 ){
                          labels.push_back(std::to_string(record.R));
                          x_axis.push_back(record.R);
                          key.insert(std::make_pair(record.R,count++));
                      } else{
                          if(x_axis.back() != record.R){
                              labels.push_back(std::to_string(record.R));
                              x_axis.push_back(record.R);
                              key.insert(std::make_pair(record.R,count++));
                          }
                      }
                  });

    // 0.1) First Iter Number
    auto iter_1  = std::vector<std::vector<int>>(key.size(), std::vector<int>());
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      if(record.fail_status == 1){
                        iter_1[ key[record.R] ].push_back(record.iter_1);
                      }
                  });

    // 0.2) t distributed - quantiles
    std::vector<double> iter1_q10;
    std::vector<double> iter1_q50;
    std::vector<double> iter1_q90;
    for ( const auto &x : x_axis){
        auto const Q_10 = iter_1[ key[x] ].size() * 0.2;
        auto const Q_50 = iter_1[ key[x] ].size() * 0.5;
        auto const Q_90 = iter_1[ key[x] ].size() * 0.8;
        std::nth_element(iter_1[ key[x] ].begin(), iter_1[ key[x] ].begin() + (int)Q_10, iter_1[ key[x] ].end());
        iter1_q10.push_back(iter_1[ key[x] ][(int)Q_10]);
        std::nth_element(iter_1[ key[x] ].begin(), iter_1[ key[x] ].begin() + (int)Q_50, iter_1[ key[x] ].end());
        iter1_q50.push_back(iter_1[ key[x] ][(int)Q_50]);
        std::nth_element(iter_1[ key[x] ].begin(), iter_1[ key[x] ].begin() + (int)Q_90,  iter_1[ key[x] ].end());
        iter1_q90.push_back(iter_1[ key[x] ][(int)Q_90]);

    }

    // 0.1) failed distributed
    auto failed_count_2 = std::vector<int>(key.size(), 0);
    auto failed_count_3 = std::vector<int>(key.size(), 0);
    auto success_count = std::vector<int>(key.size(), 0);
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&](const Record &record){
                      switch(record.fail_status){
                          case 1:
                              success_count[ key[record.R] ]++;
                              break;
                          case -2:
                              failed_count_2[ key[record.R] ]++;
                              break;
                          case -3:
                              failed_count_3[ key[record.R] ]++;
                              break;
                          default:
                              failed_count_3[ key[record.R] ]++;
                      }
                  });
    auto failed_percentage_2 = std::vector<double>(failed_count_2.size(), 0.0);
    for(auto r_id = 0; r_id < failed_count_2.size(); r_id++){
        failed_percentage_2[r_id]
                = (double)failed_count_2[r_id]*100/(failed_count_2[r_id] + failed_count_3[r_id] + success_count[r_id]);
    }
    auto failed_percentage_3 = std::vector<double>(failed_count_3.size(), 0.0);
    for(auto r_id = 0; r_id < failed_count_3.size(); r_id++){
        failed_percentage_3[r_id]
                = (double)failed_count_3[r_id]*100/(failed_count_3[r_id] +failed_count_2[r_id]+ success_count[r_id]);
    }
    auto success_percentage= std::vector<double>(success_count.size(), 0.0);
    for(auto r_id = 0; r_id < success_count.size(); r_id++){
        success_percentage[r_id]
                = (double)success_count[r_id]*100/(failed_count_2[r_id]+ failed_count_3[r_id] + success_count[r_id]);
    }

    // 1.1) t distributed - gather
    auto t_final_distributed = std::vector<std::vector<double>>( key.size(), std::vector<double>());
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&t_final_distributed, &PI, &key](const Record &record){
                      if(record.fail_status == 1){
                          t_final_distributed[ key[record.R] ].push_back(record.t);
                      }
                  });
    // 1.2) t distributed - quantiles
    std::vector<double> t_final_distributed_q10;
    std::vector<double> t_final_distributed_q50;
    std::vector<double> t_final_distributed_q90;
    for ( const auto &x : x_axis){
        auto const Q_10 = t_final_distributed[ key[x] ].size() * 0.2;
        auto const Q_50 = t_final_distributed[ key[x] ].size() * 0.5;
        auto const Q_90 = t_final_distributed[ key[x] ].size() * 0.8;
        std::nth_element(t_final_distributed[ key[x] ].begin(), t_final_distributed[ key[x] ].begin() + (int)Q_10, t_final_distributed[ key[x] ].end());
        t_final_distributed_q10.push_back(t_final_distributed[ key[x] ][(int)Q_10]);
        std::nth_element(t_final_distributed[ key[x] ].begin(), t_final_distributed[ key[x] ].begin() + (int)Q_50, t_final_distributed[ key[x] ].end());
        t_final_distributed_q50.push_back(t_final_distributed[ key[x] ][(int)Q_50]);
        std::nth_element(t_final_distributed[ key[x] ].begin(), t_final_distributed[ key[x] ].begin() + (int)Q_90,  t_final_distributed[ key[x] ].end());
        t_final_distributed_q90.push_back(t_final_distributed[ key[x] ][(int)Q_90]);

    }

    // 2.1) t setup - gather
    auto t_setup_distributed = std::vector<std::vector<double>>( key.size(), std::vector<double>());
    std::for_each(records_distributed.begin(), records_distributed.end(),
                  [&t_setup_distributed, &PI, &key](const Record &record){
                      if(record.fail_status == 1){
                          t_setup_distributed[ key[record.R] ].push_back(record.t_setup);
                      }
                  });

    // 2.2) t setup - quantiles
    std::vector<double> t_setup_distributed_q50;
    for ( const auto &x : x_axis){
        auto const Q_50 = t_setup_distributed[ key[x] ].size() * 0.5;
        std::nth_element(t_setup_distributed[ key[x] ].begin(), t_setup_distributed[ key[x] ].begin() + (int)Q_50, t_setup_distributed[ key[x] ].end());
        t_setup_distributed_q50.push_back(t_setup_distributed[ key[x] ][(int)Q_50]);
    }

    // 3 ) T setup and T final percentages
    auto t_setup_percent = std::vector<double>(t_setup_distributed_q50.size(), 0.0);
    for(auto r_id = 0; r_id < t_setup_distributed_q50.size(); r_id++){
        t_setup_percent[r_id]
                = (double)t_setup_distributed_q50[r_id]*100/(t_setup_distributed_q50[r_id] + t_final_distributed_q50[r_id]);
    }
    auto t_final_percent = std::vector<double>(t_final_distributed_q50.size(), 0.0);
    for(auto r_id = 0; r_id < t_final_distributed_q50.size(); r_id++){
        t_final_percent[r_id]
                = (double)t_final_distributed_q50[r_id]*100/(t_final_distributed_q50[r_id] + t_setup_distributed_q50[r_id]);
    }




    std::map<std::string, std::string> iter1_fill_in_between_options;
    iter1_fill_in_between_options["alpha"] = "0.2";
    iter1_fill_in_between_options["color"] = "grey";
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> iter1_median_options{
            {"color", "grey"},
            {"label", "D 50%"}
    };
    std::map<std::string, std::string> iter1_90_options =
            {
                    {"color" , "grey"},
                    {"linestyle", "dashed"},
                    {"label", "D 90%"}
            };
    std::map<std::string, std::string> iter1_10_options =
            {
                    {"color" , "grey"},
                    {"linestyle", "dashdot"},
                    {"label", "D 10%"}
            };

    std::map<std::string, std::string> distributed_fill_in_between_options;
    distributed_fill_in_between_options["alpha"] = "0.2";
    distributed_fill_in_between_options["color"] = "red";
    //fill_in_between_options["hatch"] = "-";
    std::map<std::string, std::string> distributed_median_options{
            {"color", "red"},
            {"label", "D 50%"}
    };
    std::map<std::string, std::string> distributed_90_options =
            {
                    {"color" , "red"},
                    {"linestyle", "dashed"},
                    {"label", "D 90%"}
            };
    std::map<std::string, std::string> distributed_10_options =
            {
                    {"color" , "red"},
                    {"linestyle", "dashdot"},
                    {"label", "D 10%"}
            };


    std::map<std::string, std::string> options_bar{
            //{"alpha","0.7"},
            {"color", "orange"}
    };

    std::map<std::string, std::string> success_percentage_options{
            {"color", "blue"},
    };

    std::map<std::string, std::string> t_setup_options{
            {"color", "green"},
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // Time in rows
    plt::subplot(3,1,1);
    //plt::title("Distributed");
    //plt::boxplot(t_final_distributed, labels, options);
    plt::fill_between(x_axis, t_final_distributed_q10, t_final_distributed_q90, distributed_fill_in_between_options);
    plt::plot(x_axis,t_final_distributed_q50, distributed_median_options);
    //plt::ylim(0, 20);

    /*   plt::subplot(4,1,2);
       plt::title("% Failed");
       plt::bar(failed_percentage_2,"black","-",1.25, options_bar);*/

    plt::subplot(3,1,2);
    plt::fill_between(x_axis,  iter1_q10, iter1_q90, iter1_fill_in_between_options);
    plt::plot(x_axis,iter1_q50, iter1_median_options);

    plt::subplot(3,1,3);
    plt::plot(x_axis,t_setup_percent, t_setup_options);
    plt::show();

    // plt::subplot(3,1,4);
    // plt::plot(x_axis,success_percentage, success_percentage_options);
    // plt::ylim(0,101);


    

    std::vector<std::vector<double>> failed_success = {
            success_percentage, failed_percentage_2};
    matplot::barstacked(failed_success);
    matplot::show();

    // std::vector<std::vector<double>> setup_final = {
    //         t_setup_percent, t_final_percent};
    // matplot::barstacked(setup_final);
    // matplot::show();
}

std::vector<Recorder::Record> Recorder::read_records(std::string file_name){
    std::vector<Record> records;
    std::ifstream record_file(file_name);
    while(record_file){
        std::string record_line;
        if(!getline(record_file, record_line)) break;
        // Read a record line by line
        std::stringstream record_ss(record_line);
        Record record{};
        std::string data_field;
        // R
        getline( record_ss, data_field, ',' );
        record.R = std::stoi(data_field);
        // A
        getline( record_ss, data_field, ',' );
        record.A = std::stod(data_field);
        // D
        getline( record_ss, data_field, ',' );
        record.D = std::stold(data_field);
        // L
        getline( record_ss, data_field, ',' );
        record.L = std::stod(data_field);
        // N
        getline( record_ss, data_field, ',' );
        record.N = std::stoi(data_field);
        // T
        getline( record_ss, data_field, ',' );
        record.T = std::stod(data_field);
        // cost
        getline( record_ss, data_field, ',' );
        record.cost = std::stod(data_field);
        // cost_1
        getline( record_ss, data_field, ',' );
        record.cost_1 = std::stod(data_field);
        // t
        getline( record_ss, data_field, ',' );
        record.t = std::stod(data_field);
        // t_1
        getline( record_ss, data_field, ',' );
        record.t_1 = std::stod(data_field);
        // t_setup
        getline( record_ss, data_field, ',' );
        record.t_setup = std::stod(data_field);
        // fail_status
        getline( record_ss, data_field, ',' );
        record.fail_status = std::stoi(data_field);
        // iter_1
        getline( record_ss, data_field, ',' );
        record.iter_1 = std::stoi(data_field);


        // Add Record
        records.push_back(record);
    }
    return records;
}