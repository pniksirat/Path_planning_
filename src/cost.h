#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "const.h"
#include "helpers.h"


using namespace std;

double time_diff_cost(double Real_time,double Target_T) {
  // Penalizes trajectories that is longer or shorter than targeted duration 
  return logistic(fabs(Real_time - Target_T) / Target_T);
}

double trajectory_diff_cost(vector<double> actual_traj, vector<double> target_traj){
    /*
    Penalizes trajectories whose s, d coordinate (and derivatives) 
    differ from the goal.
    actual_traj=[s,s-dot,s-dd,d,d-dot,d_dd]
    */
    
	double diff, cost;
   
    for (int i=0;i<actual_traj.size();++i){
        diff = float(abs(actual_traj[i]-target_traj[i]));
        cost += logistic(diff/SIGMA[i]);
      }
    return cost;

}



#endif