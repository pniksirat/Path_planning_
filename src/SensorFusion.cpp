#include "SensorFusion.h"

SensorFusion::SensorFusion(){
}
SensorFusion::~SensorFusion(){
}


std::pair<int, double> SensorFusion::Sensor_fusion(vector<vector<double>> SensorFusion_,Ego &Car, vector<double> prev_path_x_, vector<double> prev_path_y_ ){
  

  vector<double> previous_path_x=prev_path_x_;
  vector<double> previous_path_y=prev_path_y_;
  sensors=SensorFusion_;
  car=Car;
  
  //reset all logics for new fusion 
  got_close=false;
 
  lane_change=0; // 0: no change of lane  
  Obstacle_right=0;
  Obstacle_left=0;
  Speed_front_vehicle=99999;
  

  for (int i=0;i<sensors.size();++i){
              // cout<<"checking the cars ahead"<<endl;
               double d_vehicle=sensors[i][6];
               double vx_vehicle=sensors[i][3];
                double vy_vehicle=sensors[i][4];
                double s_vehicle=sensors[i][5];
                //use the velocity magnitude to estimate the future distance of the car since our car can be still few steps back
                //based on the previous path points, 
                double mag_speed_vehicle=sqrt(pow(vx_vehicle,2)+pow(vy_vehicle,2));
                //future S predicted
                double S_predicted_vehicle=s_vehicle+(previous_path_x.size()*delta_ti*mag_speed_vehicle);
             //is the car in the same lane (each lane 4 meter)
             //check d value for all distances inside the lane
              if(4*car.lane_status<d_vehicle && d_vehicle<4+4*car.lane_status){
                
                // if the distance from the car ahead is less than 30 meter & car is ahead of us
                // then slow down the speed
                cout<<"speed: "<<S_predicted_vehicle<<" mag "<<mag_speed_vehicle<<" path remaining: "<<previous_path_x.size()<<endl;
                if((S_predicted_vehicle-car.s<dis_from_car_ahead) && (S_predicted_vehicle>car.s) ){
                   //ref_vel=25; 
                   got_close=1;
                   Speed_front_vehicle=mag_speed_vehicle*2.24;
                   cout<<"slowing down"<<endl;
                 
                  if(mag_speed_vehicle<car.v_ref){
                  //check left lane and right lane to see if it safe to make a turn 
                  		lane_change=1;
                   }
                }  
              }
              // if on the left lane 
              else if (d_vehicle<4*car.lane_status){
                if(S_predicted_vehicle-car.s<dis_from_car_ahead && S_predicted_vehicle-car.s>-dis_from_car_back){
               	 Obstacle_left=1;
                  cout<<"obstacle right"<<endl; 
                }

              }
              
              //if on the right lane
              else if(d_vehicle>4*car.lane_status+4){
                if(S_predicted_vehicle-car.s<dis_from_car_ahead && S_predicted_vehicle-car.s>-dis_from_car_back){
                 
                Obstacle_right=1;
                cout<<"obstacle left"<<endl; 
             
                }
              } 
              
              
            }

          lane_update();
  		 // car.set_lane_Vref(car.destination_lane, car.v_ref);
   	return std::make_pair(car.lane_status,car.v_ref);
         
           }
           
          
void SensorFusion::lane_update(){
  
  
  			if(got_close){
              // if too close then slow down ~ 5 m/sec
              car.v_ref-=0.3;
              
              cout<<"v_ref in Ego"<<car.v_ref<<endl;
            }
             else if(car.v_ref<Vmax){
               car.v_ref+=0.225;   
               
               cout<<"v_ref in Ego"<<car.v_ref<<endl;
               }
  			
          
          	if (lane_change==1 && car.trajectory_status==1){
           
              // check the obstacles on right and left 
             	//check the cost for each transition 
            	cout<<"lane need to change***"<<Obstacle_right<<Obstacle_left<<endl;
               if(car.lane_status<2 && Obstacle_right==0 && Speed_front_vehicle<car.v_ref){
                 //lane++;
                 car.destination_lane= car.lane_status++; 
                 car.current_action.action="Turn_R";
                 cout<<"lane change right"<<endl; 
                 
  				}
                else if(car.lane_status==2 && Obstacle_left==0 && Speed_front_vehicle<car.v_ref){
               
                 car.destination_lane= car.lane_status--;  
                 car.current_action.action="Turn_L";
                 cout<<"lane change left"<<endl;
  				}
           }
  		 /*
  			else if (lane_change==0 &&  Obstacle_right==1 &&  Obstacle_left==1 ){
                car.current_action.action="keep_lane";
               
             }	
  
   */

}