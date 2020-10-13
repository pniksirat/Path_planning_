#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <iostream>
#include <string>
#include <vector>
#include "Ego.h"
#include <cmath>

using namespace std;
using std::string;
using std::vector;

class SensorFusion{

public:
  
  bool got_close=false;
  int lane_change=0; // 0: no change of lane  
  bool Obstacle_right=0;
  bool Obstacle_left=0;
  double Speed_front_vehicle=99999;

  vector<vector<double>> sensors;
  
  

SensorFusion();
~SensorFusion();
Ego car;

 //std::tuple<int, double> 
 std::pair<int, double> Sensor_fusion(vector<vector<double>> SensorFusion_, Ego &Car, vector<double> prev_path_x_, vector<double> prev_path_y_ );
  void lane_update();
  
};
#endif