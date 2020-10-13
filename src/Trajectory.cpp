#include "Trajectory.h"
#include <iostream>
using std::cout;
using std::endl;

/*
Trajectory::Trajectory(){

}

Trajectory::~Trajectory(){}
*/

Trajectory::Trajectory(vector<double> &map_waypoints_x,vector<double> &map_waypoints_y,vector<double> &map_waypoints_s,vector<double> &map_waypoints_dx,vector<double> &map_waypoints_dy){
  
 	map_w_pts_s=map_waypoints_s;
  	map_w_pts_dx=map_waypoints_dx;
  	map_w_pts_dy=map_waypoints_dy;
 	map_w_pts_x=map_waypoints_x;
 	map_w_pts_y=map_waypoints_y;
    //cout<<"Trajectory initialized"<<map_w_pts_s.size()<<", "<<map_w_pts_x.size()<<", "<<map_w_pts_y.size()<<endl;
  
}




vector<double> Trajectory::point_gen(double s, double d){
  
  		tk::spline Sp_dx,Sp_dy,Sp_x,Sp_y;
  
  		Sp_dx.set_points(map_w_pts_s,map_w_pts_dx);
  		Sp_dy.set_points(map_w_pts_s,map_w_pts_dy);
  		Sp_x.set_points(map_w_pts_s,map_w_pts_x);
  		Sp_y.set_points(map_w_pts_s,map_w_pts_y);

  		double dX=Sp_dx(s);
  		double dY=Sp_dy(s);
  		double X=Sp_x(s);
  		double Y=Sp_y(s);
  		
  		X=X+d*dX;
  		Y=Y+d*dY;
  
  	return {X,Y};
  		

}

void Trajectory::get_points(double l){
  
  for(int i = 1; i<4; i++){
    	
   		vector<double> spline_pts =point_gen(car_s+i*spline_dis_points,l*4+2);
    	pts_x.push_back(spline_pts[0]);
    	pts_y.push_back(spline_pts[1]);
     
  }
  
   for(int i = 0; i<pts_x.size(); i++){
                cout<<"raw pts points X:"<<pts_x[i]<<"raw Y: "<< pts_y[i]<<endl;
                
                double trans_x = pts_x[i] - ref_x;
                double trans_y = pts_y[i] - ref_y;
				
                pts_x[i] = (trans_x * cos(0-ref_yaw)-trans_y * sin(0-ref_yaw));
                pts_y[i] = (trans_x * sin(0-ref_yaw)+trans_y * cos(0-ref_yaw));
                cout<<"pts points X:"<<pts_x[i]<<"Y: "<< pts_y[i]<<endl;
     }


}


void Trajectory::Traj_intializer(){

  // Initialize points for spline 
          	// if there is only one point left on previous path find another pts tangant to car
          	//if 2 points exist on previous path then use them for the path forward as spline points
  			// Sparse points which we'll use for generating a spline
          	
  			 //cout<<"prev-path size:"<<past_path_x.size()<<endl;
            
           //reference points where the car is at if no point left from prev path
          	 ref_x = car.x;
          	 ref_y = car.y;
          	 ref_yaw = deg2rad(car.yaw);
       
            if(past_path_x.size() < 2) {
                double prev_car_x = car.x - cos(car.yaw);
                double prev_car_y = car.y - sin(car.yaw);
				
                pts_x.push_back(prev_car_x);
                pts_x.push_back(car.x);

                pts_y.push_back(prev_car_y);
                pts_y.push_back(car.y);
            	} else {
            	ref_x = past_path_x[past_path_x.size()-1];
                ref_y = past_path_y[past_path_x.size()-1];

                double ref_x_prev = past_path_x[past_path_x.size()-2];
                double ref_y_prev = past_path_y[past_path_x.size()-2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
					
                pts_x.push_back(ref_x_prev);
                pts_x.push_back(ref_x);
                pts_y.push_back(ref_y_prev);
                pts_y.push_back(ref_y);
            }
  
  
     // use the points on the previous path (un-used) 
            for(int i =0; i< past_path_x.size(); i++) {
                x_vals.push_back(past_path_x[i]);
                y_vals.push_back(past_path_y[i]);
            }
  	//cout<<"ref_x"<<ref_x<<"ref_yaw"<<ref_yaw<<endl;

}





void Trajectory::Points_on_Spline(){

	 // defining 3-points for path forward on a spline 
  			cout<<"lane status: "<<car.current_action.dest_lane<<endl;
  			
            for(int i = 1; i<4; i++){
               vector<double> spline_pts = getXY(car.s + (i*spline_dis_points),(2+4*car.current_action.dest_lane), map_w_pts_s, map_w_pts_x, map_w_pts_y);
                //vector<double> spline_pts = getXY(car_s + (i*spline_dis_points),(2+4*car.current_action.dest_lane), map_w_pts_s, map_w_pts_x, map_w_pts_y);
                pts_x.push_back(spline_pts[0]);
                pts_y.push_back(spline_pts[1]);
                
            }
			//cout<<"ref X:"<<ref_x<<"ref_y : "<< ref_y<<endl; 
            // Transform coordinates from map to car coordinates
            for(int i = 0; i<pts_x.size(); i++){
               // cout<<"raw pts points X:"<<pts_x[i]<<"raw Y: "<< pts_y[i]<<endl;
                
                double trans_x = pts_x[i] - ref_x;
                double trans_y = pts_y[i] - ref_y;
				
                pts_x[i] = (trans_x * cos(0-ref_yaw)-trans_y * sin(0-ref_yaw));
                pts_y[i] = (trans_x * sin(0-ref_yaw)+trans_y * cos(0-ref_yaw));
               // cout<<"pts points X:"<<pts_x[i]<<"Y: "<< pts_y[i]<<endl;
            }
      

}
         
vector<double> Trajectory::Plan_trajectory2(Ego &Car,double &ref_v,vector<double> prev_path_x, vector<double> prev_path_y)
{			
  
  			car=Car;
			car_s=car.s;
  			past_path_x=prev_path_x;
    		past_path_y=prev_path_y;
  			double velocity_ref=ref_v;
  			double lane=car.destination_lane;
  			//ref_vel=car.v_ref;
  			//cout<<"pts points s:"<<car.s<<"v: "<< car.v_ref<<endl;
  			cout<<"v_ref in Traj"<<car.v_ref<<endl;
  			cout<<"lane status: "<<car.lane_status<<endl;
  			cout<<"lane destination: "<<lane<<endl;
  			
  			
  
  			//set the two first point for the spline and the ref yaw
  			Traj_intializer();
  			//generate some points for spline
  			get_points(lane);
  
  			

             //create a spline based on 2 points from previous path and 3 new points defined above
            tk::spline s;
            s.set_points(pts_x, pts_y);
 			
  

            // This calculates how many points we need in the spline so we travel at the desired speed
            // by approximating the length of the segment using a straight line
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            //For each speed, number of points that defines the specific distance (target_dist)
            double N = target_dist / (delta_ti * velocity_ref/2.24);

            double x_ref_OnSpline = 0;
            // add the spline points to the vector that will be sent to simulation 
            for(int i = 1; i <= 50-past_path_x.size(); i++) {
                
              
                double x_point = x_ref_OnSpline + target_x / N;
                double y_point = s(x_point);
				//each point of spline starts from the previous point
                x_ref_OnSpline = x_point;
				
                
                double x_ref = x_point;
                double y_ref = y_point;

                // transform from car coordinates to map coordinates
                x_point = ref_x+ (x_ref* cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = ref_y+ (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));



                x_vals.push_back(x_point);
                y_vals.push_back(y_point);
            }


 return x_vals,y_vals;
}


//vector<double> Trajectory::Plan_trajectory(Ego &Car,vector<double> prev_path_x, vector<double> prev_path_y)  
vector<double> Trajectory::Plan_trajectory(Ego &Car,double &ref_v,vector<double> prev_path_x, vector<double> prev_path_y)
{			
  
  			car=Car;
			car_s=car.s;
  			past_path_x=prev_path_x;
    		past_path_y=prev_path_y;
  			double velocity_ref=ref_v;
  			double lane=car.destination_lane;
  			//ref_vel=car.v_ref;
  			//cout<<"pts points s:"<<car.s<<"v: "<< car.v_ref<<endl;
  			cout<<"v_ref in Traj"<<car.v_ref<<endl;
  			cout<<"lane status: "<<car.lane_status<<endl;
  			cout<<"lane destination: "<<lane<<endl;
  			
  			
  
  			//set the two first point for the spline and the ref yaw
  			Traj_intializer();
  			//generate some points for spline
  			//Points_on_Spline();
  
  			 // defining 3-points for path forward on a spline 
  			//cout<<"lane status: "<<car.lane_status<<endl;
 
  		   
            for(int i = 1; i<4; i++){
               // vector<double> spline_pts = getXY(car_s + (i*spline_dis_points),(2+4*car.current_action.dest_lane), map_w_pts_s, map_w_pts_x, map_w_pts_y);
                vector<double> spline_pts = getXY(car_s + (i*spline_dis_points),(2+4*lane), map_w_pts_s, map_w_pts_x, map_w_pts_y);
                pts_x.push_back(spline_pts[0]);
                pts_y.push_back(spline_pts[1]);
               // cout<<"raw pts points X:"<<pts_x[i]<<"raw Y: "<< pts_y[i]<<endl;
            }

            // Transform coordinates from map to car coordinates
            for(int i = 0; i<pts_x.size(); i++){
              
                 
                double trans_x = pts_x[i] - ref_x;
                double trans_y = pts_y[i] - ref_y;
				
                pts_x[i] = (trans_x * cos(0-ref_yaw)-trans_y * sin(0-ref_yaw));
                pts_y[i] = (trans_x * sin(0-ref_yaw)+trans_y * cos(0-ref_yaw));
               // cout<<"pts points X:"<<pts_x[i]<<"Y: "<< pts_y[i]<<endl;
            }
      

  
  		

             //create a spline based on 2 points from previous path and 3 new points defined above
            tk::spline s;
            s.set_points(pts_x, pts_y);
 			
  

            // This calculates how many points we need in the spline so we travel at the desired speed
            // by approximating the length of the segment using a straight line
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            //For each speed, number of points that defines the specific distance (target_dist)
            double N = target_dist / (delta_ti * velocity_ref/2.24);

            double x_ref_OnSpline = 0;
            // add the spline points to the vector that will be sent to simulation 
            for(int i = 1; i <= 50-past_path_x.size(); i++) {
                
              
                double x_point = x_ref_OnSpline + target_x / N;
                double y_point = s(x_point);
				//each point of spline starts from the previous point
                x_ref_OnSpline = x_point;
				
                
                double x_ref = x_point;
                double y_ref = y_point;

                // transform from car coordinates to map coordinates
                x_point = ref_x+ (x_ref* cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = ref_y+ (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));



                x_vals.push_back(x_point);
                y_vals.push_back(y_point);
            }
  //cout<<"next x vals "<<x_vals[10]<<"next y vals "<< y_vals[10]<<endl;

 	// car.lane_status=car.current_action.dest_lane;
 	// car.trajectory_status=1; //done the trjectory
  
  return x_vals,y_vals;
}