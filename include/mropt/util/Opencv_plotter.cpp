#include "Opencv_plotter.hpp"

#include <random>
#include <algorithm>

using namespace cv;
using namespace mropt::util;
void Opencv_plotter::addLine(Mat img, Point start, Point end)
{
    int thickness = 2;
    int lineType = LINE_8;
    line(img,
         start,
         end,
         Scalar(255, 0, 0),
         thickness,
         lineType);
}

void Opencv_plotter::addFilledCircle(Mat img, Point center, int r_id)
{
    circle(img,
           center,
           length_px_[r_id] / 2,
           colors[r_id],
           FILLED,
           LINE_8);
}

void Opencv_plotter::addRectangle(Mat img, Point corner1, Point corner2)
{
    cv::rectangle(img,
                  corner1,
                  corner2,
                  Scalar(100, 100, 50));
}

void Opencv_plotter::fillRectangle(Mat img, Point corner1, Point corner2)
{
    cv::rectangle(img,
                  corner1,
                  corner2,
                  Scalar(0, 255, 0),
                  -1);
}

void Opencv_plotter::whiteRectangle(Mat img, Point corner1, Point corner2)
{
    cv::rectangle(img,
                  corner1,
                  corner2,
                  Scalar(255, 255, 255),
                  -1);
}

void Opencv_plotter::blueRectangle(Mat img, Point corner1, Point corner2)
{
    cv::rectangle(img,
                  corner1,
                  corner2,
                  Scalar(30, 210, 100));
}

void Opencv_plotter::addConvexPolygon(Mat img, const Point *points, int n_pts)
{
    fillPoly(img,
             &points,
             &n_pts,
             1,
             Scalar(100, 100, 50),
             LINE_8);

    polylines(img,
              &points,
              &n_pts,
              1,
              true,
              Scalar(240, 50, 100));
}

void Opencv_plotter::fillPolygonBlack(Mat img, const Point *points, int n_pts)
{
    fillPoly(img,
             &points,
             &n_pts,
             1,
             Scalar(0, 255, 0),
             LINE_8);
}

void Opencv_plotter::createRectangle(
        const Point2d &center,
        int pos_x, int neg_x,
        int pos_y, int neg_y,
        std::shared_ptr<Rectangle> &rect)
{
    rect->left_bottom_corner = Point2d(center.x - neg_x, center.y - neg_y);
    rect->right_upper_corner = Point2d(center.x + pos_x, center.y + pos_y);
}

void Opencv_plotter::rectangleAngle(Mat img, int x0, int y0, int width, int height, double angle)
{
    auto _angle = angle * M_PI / 180.0;

    double b = std::cos(_angle) * 0.5;
    double a = std::sin(_angle) * 0.5;
    auto pt0 = Point(int(x0 - a * height - b * width),
                     int(y0 + b * height - a * width));
    auto pt1 = Point(int(x0 + a * height - b * width),
                     int(y0 - b * height - a * width));
    auto pt2 = Point(int(2 * x0 - pt0.x), int(2 * y0 - pt0.y));
    auto pt3 = Point(int(2 * x0 - pt1.x), int(2 * y0 - pt1.y));

    line(img, pt0, pt1, (255, 255, 255), 2);
    line(img, pt1, pt2, (255, 255, 255), 2);
    line(img, pt2, pt3, (255, 255, 255), 2);
    line(img, pt3, pt0, (255, 255, 255), 2);
}

void Opencv_plotter::setFootprint(std::vector<double> length, std::vector<double> width)
{
    for (int r_id = 0; r_id < R; r_id++)
    {
        this->length_px_[r_id] = length[r_id] * resolution; //TODO resolution 10 et this from somewhere
        this->width_px_[r_id] = width[r_id] * resolution;   //TODO resolution 10 et this from somewhere
    }
}

double Opencv_plotter::area(const Rectangle &rect)
{
    return (rect.left_bottom_corner.x - rect.right_upper_corner.x) *
           (rect.left_bottom_corner.x - rect.right_upper_corner.x) +
           (rect.left_bottom_corner.y - rect.right_upper_corner.y) *
           (rect.left_bottom_corner.y - rect.right_upper_corner.y);
}


void Opencv_plotter::plot_diffdrive_robot(
        double x_px, double y_px, double o,
        int r_id, double length_px,
        Mat &color_img_)
{
    cv::Mat overlay;
    double alpha = 0.3;
    color_img_.copyTo(overlay);

    // Main circle Robot

    circle(overlay,
           cv::Point(x_px, y_px),
           length_px / 2,
           colors[r_id],
           FILLED,
           LINE_8);

    //rectangleAngle(color_img, x_px + 0.4 * length_px * std::cos(o - M_PI / 2), y_px + 0.4 * length_px * (std::sin(o - M_PI / 2)), 5, 5, o * 180 / M_PI);
    // Left wheel
    circle(color_img_,
           cv::Point(x_px + 0.45 * length_px * std::cos(o - M_PI / 2),
                     y_px + 0.55 * length_px * (std::sin(o - M_PI / 2))),
           4,
           Scalar(0, 0, 0), //colors[r_id]
           FILLED);

    //rectangleAngle(color_img, x_px + 0.4 * length_px * std::cos(o + M_PI / 2), y_px + 0.4 * length_px * (std::sin(o + M_PI / 2)), 5, 5, o * 180 / M_PI);
    // RIght wheel
    circle(color_img_,
           cv::Point(x_px + 0.45 * length_px * std::cos(o + M_PI / 2),
                     y_px + 0.55 * length_px * (std::sin(o + M_PI / 2))),
           4,
           Scalar(0, 0, 0), //colors[r_id]
           FILLED);

    auto center = cv::Point(x_px + 0.5 * length_px * std::cos(o), y_px + 0.5 * length_px * (std::sin(o)));


    circle(overlay,
           center,
           4,
           Scalar(255, 255, 255),
           LINE_AA); //4


/*    circle(color_img_,
           cv::Point(x_px, y_px),
           (length_px / 2),
           Scalar(255, 255, 255),
           1);*/

    //circle(overlay,
    //       cv::Point(x_px, y_px),
    //       length_px/ 2,
    //       Scalar(0, 0, 0),
    //       LINE_AA);//8

    cv::addWeighted(
            overlay, alpha,
            color_img_, 1 - alpha, 0, color_img_);
}

void Opencv_plotter::plot_trajectory(
        std::vector<std::vector<double>> x,
        std::vector<std::vector<double>> y,
        std::vector<std::vector<double>> o,
        std::vector<std::vector<std::vector<double>>> u,
        std::vector<double> time)  {
    plot_trajectory(x,y,o,u, time[0]);//TODO: it is not working under this roboots are slow and stuck, this works if all time vector is equal
    return;
    // 0 - Setup
    auto N = std::vector<int>(R,0);
    auto k = std::vector<int>(R,0);
    auto dt_curr = std::vector<double>(R,0);
    auto dt = std::vector<double>(R,0);
    int robots_finished = 0;

    std::vector<std::chrono::high_resolution_clock::time_point > start
            =  std::vector<std::chrono::high_resolution_clock::time_point>(R);
    std::chrono::high_resolution_clock::time_point  end;
    for (int r = 0; r < R; r++) {
        N[r] = x[r].size();
        k[r] = 0;
        dt[r] = time[r]/N[r];
        dt_curr[r] = 0;
        start[r] = std::chrono::high_resolution_clock::now();
    }

    while(robots_finished != R) {
        for (int r = 0; r < R; r++) {
            if(k[r] == N[r]-1){//standstill
                //plot_robot[r](x[r][N[r]], y[r][N[r]], o[r][N[r]]);
            } else {
                DM ur = DM({u[r]});
                DM X_sol = rk4_num(odes[r]->f(), DM(dt_curr[r]), DM{x[r][k[r]], y[r][k[r]], o[r][k[r]]}, ur(all, k[r]));
                plot_robot[r](x[r][k[r]] + X_sol(0).scalar(),
                              y[r][k[r]] + X_sol(1).scalar(),
                              o[r][k[r]] + X_sol(2).scalar());
            }
        }
        end = std::chrono::high_resolution_clock::now();


        for(int r = 0; r < R; r++){
            if(k[r] == N[r]-1) continue;
            //dt_curr[r] = (double(end - start[r]) / (double) CLOCKS_PER_SEC);
            auto dt_curr_d = end-start[r];
            dt_curr[r] =(double) dt_curr_d.count()/1000000000;
            if(dt_curr[r] > dt[r]){//Finished a discrete step
                k[r]++;
                dt_curr[r] = 0;
                start[r] =  std::chrono::high_resolution_clock::now();
                if(k[r] == N[r]-1) robots_finished++;
            }
        }
        imshow("Trajectory", color_img);
        waitKey(1);
    }
    waitKey();
}

