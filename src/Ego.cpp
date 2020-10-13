#include "Ego.h"
#include <cmath>

Ego::Ego() {
 	lane_status=1;
  	destination_lane=1;
    current_action.action="keep_lane";
    current_action.dest_lane=1;
  	current_action.origin_lane=1;
  	v_ref=0;
  	trajectory_status=1;
    

}

Ego::~Ego() {
}

 void Ego::set_ego(double X, double Y, double S, double D, double Yaw, double Speed){
  	x=X;
  	y=Y;
	s=S;
  	d=D;
  	yaw=Yaw;
  	speed=Speed;
  	lane_status=int(floor(d/4)); 
  //trajectory_status
    if (current_action.action=="keep_lane" || lane_status==current_action.dest_lane){
    	trajectory_status=1;
      	current_action.dest_lane=lane_status;
      	current_action.origin_lane=lane_status;
    }
    else {
          trajectory_status=0; 
 	}
  }


void Ego::set_lane_Vref(int L, double v_r){

  destination_lane=L;
  v_ref=v_r;

}
                   
//void Ego::State_update(){}                   