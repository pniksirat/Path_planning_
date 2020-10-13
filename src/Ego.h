#ifndef EGO_H
#define EGO_H

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using std::string;
using std::vector;

class Ego{

public:
  double x,y,s,d,yaw,speed;
  bool trajectory_status; //completed or not
  int lane_status;
  //int updated_lane;
  bool got_close=false;
  int lane_change=0; // 0: no change of lane  
  bool Obstacle_right=0;
  bool Obstacle_left=0;
  double Speed_front_vehicle=99999;
  double v_ref;
  int destination_lane;
  vector<vector<double>> sensors;
  
  
  
  
  
  struct potential_action{
    string action;
    int origin_lane;
    int dest_lane;
  };
  
  vector<potential_action> potential_actions;
  potential_action current_action;
  
  Ego();
  ~Ego();
  
  void set_ego(double x, double y, double s, double d, double yaw, double speed);
  void set_lane_Vref(int L, double v_r);

  
  void lane_update();
  		
};




#endif