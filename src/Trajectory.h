#ifndef TRAJECTORY_H
#define TRAJECTORY_H


#include <vector>
#include <math.h>  
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "const.h"
#include "Ego.h"


using namespace std;
using std::string;
using std::vector;

class Trajectory{
  
  public:
  
   vector<double> pts_x;
   vector<double> pts_y;
   double ref_x;
   double ref_y;
   double ref_yaw;
   double car_s;
   vector<double> past_path_x,past_path_y;
   double ref_vel;
   vector<double> map_w_pts_s,map_w_pts_dx,map_w_pts_dy,map_w_pts_x, map_w_pts_y;
   vector<double> x_vals,y_vals; //x, y values to pass to simulators
  
  Ego car;
  //tk::spline Sp_dx,Sp_dy,Sp_x,Sp_y;
  
  //Trajectory();  
  Trajectory(vector<double> &map_waypoints_x,vector<double> &map_waypoints_y,vector<double> &map_waypoints_s,vector<double> &map_waypoints_dx,vector<double> &map_waypoints_dy);
  //~Trajectory();  
  
  void Points_on_Spline();
  void Traj_intializer();
  

 vector<double> Plan_trajectory(Ego &Car,double &ref_v,vector<double> prev_path_x, vector<double> prev_path_y);
 
  vector<double> Plan_trajectory2(Ego &Car,double &ref_v,vector<double> prev_path_x, vector<double> prev_path_y);
  
  //vector<double> Plan_trajectory(Ego &Car,vector<double> prev_path_x, vector<double> prev_path_y);
vector<double> point_gen(double s, double d);   //function spline based on Wps
void get_points(double l);

};

#endif