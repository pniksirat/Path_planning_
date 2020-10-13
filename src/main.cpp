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
#include "SensorFusion.h"
#include "SensorFusion.cpp"

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
  vector<double> map_waypoints_dx; //normal vector pointing outward of the highway loop.
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
  
  Ego self_driver;
  SensorFusion Sense;

  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&self_driver,&Sense]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
                
    //freopen( "output.txt", "w", stdout );
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
          double Speed_front_vehicle=99999;
          
          
            Trajectory traj(map_waypoints_x,map_waypoints_y,map_waypoints_s,map_waypoints_dx,map_waypoints_dy);

          json msgJson;
          
          
          
          if(previous_path_x.size() > 0) {
                car_s = end_path_s;
          	}
          if (check_lane(self_driver.destination_lane,car_d)){
 	 			self_driver.lane_status=self_driver.destination_lane;
 	 			self_driver.trajectory_status=1; //done the trjectory
                cout<<"previous traj completed"<<endl;
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
          
          	self_driver.set_ego(car_x,car_y,car_s,car_d,car_yaw,car_speed);
           
          	//cout<<"Trajectory initialized"<<map_waypoints_s[100]<<endl;
            
        /*
            //read the sensor data for all cars [id,x,y,vx,vy,s,d]
            for (int i=0;i<sensor_fusion.size();++i){
              // cout<<"checking the cars ahead"<<endl;
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
                   Speed_front_vehicle=mag_speed_vehicle*2.24;
                   //cout<<"slowing down"<<endl;
                 
                  if(Speed_front_vehicle<ref_vel){
                  //check left lane and right lane to see if it safe to make a turn 
                  		lane_change=1;
                   }
                }  
              }
              // if on the right lane 
              else if (d_vehicle<4*lane){
                if(S_predicted_vehicle-car_s<dis_from_car_ahead && S_predicted_vehicle-car_s>-dis_from_car_back){
               	 Obstacle_left=1;
                  cout<<"obstacle left"<<endl; 
                }

              }
              
              //if on the right lane
              else if(d_vehicle>4*lane+4){
                if (S_predicted_vehicle-car_s<dis_from_car_ahead && S_predicted_vehicle-car_s>-dis_from_car_back){
                Obstacle_right=1;
                cout<<"obstacle right"<<endl; 
                 }
             	}
            }  
              
            
          
			if(got_close){
              // if too close then slow down ~ 5 m/sec
              ref_vel-=0.3;
              self_driver.v_ref=ref_vel;
              cout<<"v_ref in MAIN"<<self_driver.v_ref<<endl;
            }
             else if(ref_vel<Vmax){
               ref_vel+=0.225;   
               self_driver.v_ref=ref_vel;
               cout<<"v_ref in MAIN"<<self_driver.v_ref<<endl;
               }
          
          	if (lane_change==1){// && self_driver.trajectory_status==1){
           		// check the obstacles on right and left 
             	//check the cost for each transition 
            	cout<<"lane need to change***"<<Obstacle_right<<Obstacle_left<<endl;
               if(lane<2 && Obstacle_right==0 && Speed_front_vehicle<ref_vel){
                 lane++;
                 self_driver.destination_lane= lane; 
                 self_driver.current_action.action="Turn_R";
                 cout<<"lane change right"<<endl; 
                 
  				}
                else if(lane==2 && Obstacle_left==0 && Speed_front_vehicle<ref_vel){
                 lane--;
                 self_driver.destination_lane= lane;  
                 self_driver.current_action.action="Turn_L";
                 cout<<"lane change left"<<endl;
  				}
           }
           
          cout<<"Lane in MAIN"<<lane<<endl;
        */  
    
        std::pair<int, double> R=Sense.Sensor_fusion(sensor_fusion, self_driver, previous_path_x, previous_path_y);
        ref_vel= R.second;
        self_driver.v_ref= R.second;
        self_driver.destination_lane=R.first;
        cout<<"Main vref ******"<< R.second<<"lane"<<R.first<<endl; 
        //self_driver.destination_lane;
          
          
         traj.Plan_trajectory(self_driver,ref_vel, previous_path_x, previous_path_y);
         //traj.Plan_trajectory2(self_driver,ref_vel, previous_path_x, previous_path_y);

          //cout<<"next x vals main "<<next_x_vals[10]<<"next y vals "<< next_y_vals[10]<<endl;
          next_x_vals=traj.x_vals;
          next_y_vals=traj.y_vals;
       
          //Note:deleted the class from here 

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
    //fclose (stdout);
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