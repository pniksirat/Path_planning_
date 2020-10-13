#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>  
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "jmt.h"
#include "cost.h"
#include "const.h"
#include "Ego.h"
#include "Ego.cpp"
#include "Trajectory.h"
#include "Trajectory.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
   double ref_vel=0.0;
  /*
   static const double delta_ti = 0.02;
   static const double dis_from_car_ahead=30;

   static const double spline_dis_points=30;
   
  */ 
  //car starts from lane 1      	
   int lane = 1;
  
  Ego self_diver;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&self_diver]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
          self_diver.set_ego(car_x,car_y,car_s,car_d,car_yaw,car_speed);
          Trajectory traj(map_waypoints_x,map_waypoints_y,map_waypoints_s);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          bool got_close=false;
          bool kpl=true; // keep lane
          int lane_change=0; // 0: no change of lane  
          bool Obstacle_right=0;
          bool Obstacle_left=0;
          

          json msgJson;
          
          
          
          if(previous_path_x.size() > 0) {
                car_s = end_path_s;
          	}

			//vector of points that sends to simulator
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	// Sparse points which we'll use for generating a spline
          	vector<double> pts_x;
          	vector<double> pts_y;
            
           //reference points where the car is at if no point left from prev path
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          
           
            //read the sensor data for all cars [id,x,y,vx,vy,s,d]
            for (int i=0;i<sensor_fusion.size();++i){
               cout<<"checking the cars ahead"<<endl;
               double d_vehicle=sensor_fusion[i][6];
               double vx_vehicle=sensor_fusion[i][3];
                double vy_vehicle=sensor_fusion[i][4];
                double s_vehicle=sensor_fusion[i][5];
                //use the velocity magnitude to estimate the future distance of the car since our car can be still few steps back
                //based on the previous path points, 
                double mag_speed_vehicle=sqrt(pow(vx_vehicle,2)+pow(vy_vehicle,2));
                //future S predicted
                double S_predicted_vehicle=s_vehicle+(previous_path_x.size()*delta_ti*mag_speed_vehicle);
             //is the car in the same lane (each lane 4 meter)
             //check d value for all distances inside the lane
              if(4*lane<d_vehicle && d_vehicle<4+4*lane){
                
                // if the distance from the car ahead is less than 30 meter & car is ahead of us
                // then slow down the speed
                cout<<"speed: "<<S_predicted_vehicle<<" mag "<<mag_speed_vehicle<<" path remaining: "<<previous_path_x.size()<<endl;
                if((S_predicted_vehicle-car_s<dis_from_car_ahead) && (S_predicted_vehicle>car_s) ){
                   //ref_vel=25; 
                   got_close=1;
                   cout<<"slowing down"<<endl;
                 }
                  if(mag_speed_vehicle<45){
                  //check left lane and right lane to see if it safe to make a turn 
                  		lane_change=1;
                   }
                
              }
              // if on the right lane 
              if (d_vehicle<4*lane){
                if((S_predicted_vehicle-car_s<dis_from_car_ahead) || (S_predicted_vehicle-car_s<-dis_from_car_ahead)){
               	 Obstacle_right=1;
                }

              }
              
              //if on the left lane
              if(d_vehicle>4*lane+4 && (S_predicted_vehicle-car_s<dis_from_car_ahead || S_predicted_vehicle-car_s<-dis_from_car_ahead)){
                Obstacle_left=1;
              }
              
              
            }
          
			if(got_close){
              // if too close then slow down ~ 5 m/sec
              ref_vel-=0.25;
            }
             else if(ref_vel<49.25){
               ref_vel+=0.25;   
               }
        
          
          
         
             //if not and the the car is slowed down speed up again
          
          	// Initialize points for spline 
          	// if there is only one point left on previous path find another pts tangant to car
          	//if 2 points exist on previous path then use them for the path forward as spline points
       
            if(previous_path_x.size() < 2) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                pts_x.push_back(prev_car_x);
                pts_x.push_back(car_x);

                pts_y.push_back(prev_car_y);
                pts_y.push_back(car_y);
            } else {
            	ref_x = previous_path_x[previous_path_x.size()-1];
                ref_y = previous_path_y[previous_path_x.size()-1];

                double ref_x_prev = previous_path_x[previous_path_x.size()-2];
                double ref_y_prev = previous_path_y[previous_path_x.size()-2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                pts_x.push_back(ref_x_prev);
                pts_x.push_back(ref_x);
                pts_y.push_back(ref_y_prev);
                pts_y.push_back(ref_y);
            }

            // defining 3-points for path forward on a spline 
            for(int i = 1; i<4; i++){
                vector<double> spline_pts = getXY(car_s + (i*spline_dis_points),(2+4*lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
                pts_x.push_back(spline_pts[0]);
                pts_y.push_back(spline_pts[1]);
            }

            // Transform coordinates from map to car coordinates
            for(int i = 0; i<pts_x.size(); i++){
                double shift_x = pts_x[i] - ref_x;
                double shift_y = pts_y[i] - ref_y;

                pts_x[i] = (shift_x * cos(0-ref_yaw)-shift_y * sin(0-ref_yaw));
                pts_y[i] = (shift_x * sin(0-ref_yaw)+shift_y * cos(0-ref_yaw));
            }
			
            //create a spline based on 2 points from previous path and 3 new points defined above
            tk::spline s;
            s.set_points(pts_x, pts_y);



            // Reuse previous points
            for(int i =0; i< previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // This calculates how many points we need in the spline so we travel at the desired speed
            // by approximating the length of the segment using a straight line
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            //For each speed, number of points that defines the specific distance (target_dist)
            double N = target_dist / (delta_ti * ref_vel/2.24);

            double x_ref_OnSpline = 0;
            // add the spline points to the vector that will be sent to simulation 
            for(int i = 1; i <= 50-previous_path_x.size(); i++) {
                double x_point = x_ref_OnSpline + target_x / N;
                double y_point = s(x_point);
				//each point of spline starts from the previous point
                x_ref_OnSpline = x_point;
				
                
                double x_ref = x_point;
                double y_ref = y_point;

                // transform from car coordinates to map coordinates
                x_point = ref_x+ (x_ref* cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = ref_y+ (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));



                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

  		  msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}