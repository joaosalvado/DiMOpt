#ifndef OPENCV_PLOTTER_H
#define OPENCV_PLOTTER_H
#pragma once

#include "mropt/util/Plotter.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Core>
#include <ctime>

#include <iostream>

using namespace cv;

#include <chrono>

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace mropt::util {
    class Opencv_plotter : public mropt::util::Plotter {
        std::vector<cv::Scalar> colors =
                {
                        Scalar(230, 25, 75), Scalar(60, 180, 75), Scalar{255, 225, 25},
                        Scalar{0, 130, 200}, Scalar(240, 50, 230),
                        Scalar(210, 245, 60), Scalar(250, 190, 212), Scalar(0, 128, 128),
                        Scalar(220, 190, 255), Scalar(170, 110, 40), Scalar(255, 250, 200),
                        Scalar(128, 0, 0), Scalar(170, 255, 195), Scalar(128, 128, 0),
                        Scalar(255, 215, 180), Scalar(0, 0, 128), Scalar(128, 128, 128),
                        Scalar(0, 0, 64), Scalar(0, 64, 0), Scalar(64, 0, 0),
                        Scalar(0, 64, 64), Scalar(64, 0, 64), Scalar(64, 64, 0),
                        Scalar(0, 0, 192), Scalar(0, 192, 0), Scalar(192, 0, 0),
                        Scalar(0, 192, 192), Scalar(192, 0, 192), Scalar(192, 192, 0),
                        Scalar(0, 0, 0)
                };
        typedef struct rectangle_ {
            Point2d left_bottom_corner;
            Point2d right_upper_corner;
        } Rectangle;

    public:
        explicit Opencv_plotter(int R_, int pol_n = 0)
                : mropt::util::Plotter(R_, pol_n),
                  length_px_(std::vector<int>(R_)),
                  width_px_(std::vector<int>(R_)) {

            for (int r_id = 0; r_id < R; ++r_id) {
                plot_robot.emplace_back(
                        [&, r_id](std::vector<double> xall) { return plot_diffdrive_robot(xall, r_id); });
            }
        }

        virtual ~Opencv_plotter() {};

    private:
        Mat color_img, gray_img;
        std::string image_path, maps_path;
        std::vector<int> length_px_, width_px_;
        double resolution = 100;

        double area(const Rectangle &rect);

        void createRectangle(
                const Point2d &center,
                int pos_x, int neg_x,
                int pos_y, int neg_y,
                std::shared_ptr<Rectangle> &rect);

        //Images
        void addConvexPolygon(Mat img, const Point *points, int n_pts);

        void addRectangle(Mat img, Point corner1, Point corner2);

        void addFilledCircle(Mat img, Point center, int length_px);

        void addLine(Mat img, Point start, Point end);

        void fillPolygonBlack(Mat img, const Point *points, int n_pts);

        void fillRectangle(Mat img, Point corner1, Point corner2);

        void whiteRectangle(Mat img, Point corner1, Point corner2);

        void blueRectangle(Mat img, Point corner1, Point corner2);

        void rectangleAngle(Mat img, int x, int y, int width, int height, double angle);

    public:
        void plot_trajectory(
                std::vector<std::vector<double>> x,
                std::vector<std::vector<double>> y,
                std::vector<std::vector<double>> o,
                std::vector<std::vector<std::vector<double>>> u,
                std::vector<double> time);

        /*void plot_trajectory(
            std::vector<std::vector<double>> x,
            std::vector<std::vector<double>> y,
            std::vector<std::vector<double>> o,
            std::vector<std::vector<std::vector<double>>> u,
            double time) override {
          auto N = x[0].size();
          double dt = time / N;
          for (int k = 0; k < N - 1; ++k) {
            //clock_t start, end;
            double dt_curr = 0;
            auto start = std::chrono::high_resolution_clock::now();//clock();
            do {

              for (int r = 0; r < R; r++) {
                DM ur = DM({u[r]});
                DM X_sol = rk4_num(odes[r]->f(), DM(dt_curr), DM{x[r][k], y[r][k], o[r][k]}, ur(all, k));
                plot_robot[r]({x[r][k] + X_sol(0).scalar(), y[r][k] + X_sol(1).scalar(), o[r][k] + X_sol(2).scalar()});
              }
              auto end = std::chrono::high_resolution_clock::now();
              imshow("Trajectory", color_img);
              //dt_curr = (double(end - start) / (double) CLOCKS_PER_SEC);
              auto dt_curr_d = end-start;
              dt_curr =(double) dt_curr_d.count()/1000000000;
              waitKey(1);

            } while (dt_curr < dt);
            //waitKey();
          }
          waitKey();

          // Clear trajectories
          color_img = Scalar(255,255,255); //cv::Mat::zeros(color_img.rows, color_img.cols, color_img.type());
        }*/

        void plot_trajectory(
                std::vector<std::vector<std::vector<double>>> x,
                std::vector<std::vector<std::vector<double>>> u,
                double time) override;

        void plot_diffdrive_robot(std::vector<double> xall, int r_id) {
            double x, y, o;
            odes[r_id]->state_space_->getSE2(xall, x, y, o);
            auto length_px = length_px_[r_id];
            auto x_px = x * resolution;
            auto y_px = y * resolution;

            cv::Mat overlay;
            double alpha = 0.05;
            color_img.copyTo(overlay);

            // Main circle robot

            addFilledCircle(overlay, cv::Point(x * resolution, y * resolution), r_id);
            //rectangleAngle(color_img, x_px + 0.4 * length_px * std::cos(o - M_PI / 2), y_px + 0.4 * length_px * (std::sin(o - M_PI / 2)), 5, 5, o * 180 / M_PI);
            // Right Wheel
            circle(color_img,
                   cv::Point(x_px + 0.49 * length_px * std::cos(o - M_PI / 2),
                             y_px + 0.49 * length_px * (std::sin(o - M_PI / 2))),
                   1,
                   Scalar(0, 0, 0), //colors[r_id],
                   FILLED);

            //rectangleAngle(color_img, x_px + 0.4 * length_px * std::cos(o + M_PI / 2), y_px + 0.4 * length_px * (std::sin(o + M_PI / 2)), 5, 5, o * 180 / M_PI);
            // Left Wheel
            circle(color_img,
                   cv::Point(x_px + 0.49 * length_px * std::cos(o + M_PI / 2),
                             y_px + 0.49 * length_px * (std::sin(o + M_PI / 2))),
                   1,
                   Scalar(0, 0, 0), //colors[r_id],
                   FILLED);


            auto center = cv::Point(x_px + 0.5 * length_px * std::cos(o), y_px + 0.5 * length_px * (std::sin(o)));

            //circle(overlay,
            //       center,
            //       2,
            //       Scalar(255, 255, 255),
            //       LINE_4);
            cv::arrowedLine(
                    color_img,
                    cv::Point(x_px, y_px),
                    center,
                    Scalar(0, 0, 0),
                    1, LINE_AA, 0, 0.2);


            //circle(color_img,
            //       cv::Point(x * resolution, y * resolution),
            //       length_px_[r_id] / 2,
            //       Scalar(0, 0, 0),
            //       LINE_8);
/*
      circle(color_img,
             cv::Point(x_px, y_px),
             (length_px / 2),
             Scalar(255, 255, 255),
             5);*/


            cv::addWeighted(
                    overlay, alpha,
                    color_img, 1 - alpha, 0, color_img);

        }

        void plot_diffdrive_robot(
                double x, double y, double o,
                int r_id, double length_px,
                Mat &color_img_
        );

        void setFootprint(std::vector<double> Lenght, std::vector<double> width);

        void inputScenario(const std::string &file_name) {
            //READ IMAGE
            //image_path = samples::findFile(file_name);
            std::stringstream image_path_ss;
            image_path_ss << maps_path << file_name;
            image_path = image_path_ss.str();
            color_img = imread(image_path, IMREAD_COLOR);
            cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

            if (color_img.empty()) {
                std::cout << "Could not read the image: " << image_path << std::endl;
                return;
            }
        }

        void addPathToScenarios(const std::string &maps_folder_path) {
            samples::addSamplesDataSearchPath(maps_folder_path);
            maps_path = maps_folder_path;
        }

        void resetImage() {
            color_img = imread(image_path, IMREAD_COLOR);
            cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

            if (color_img.empty()) {
                std::cout << "Could not read the image: " << image_path << std::endl;
                return;
            }
        }

        void disp() {
            cv::imshow("Display win.dow2", color_img);
            waitKey();
        }
    };
}
#endif