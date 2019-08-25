#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  
  // Define variables for lane changing
  int ego_lane = 1;
  
  // Set target velocity in mph to zero for starting smoothly
  double ref_vel = 0.0;

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ego_lane]
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
          double ego_vehicle_s = j[1]["s"];
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
          
          // Define previous path size
          int prev_size = previous_path_x.size();
          
          // Define s value for the Ego vehicle
          if(prev_size > 0){
            ego_vehicle_s = end_path_s;
          }
          
          // Initialize behavior states to false
          // States of surrounding vehicles define the State of the Ego vehicle
          // Specific State combinations will dictate Ego vehicle actions
          bool too_close = false;
          bool vehicle_left = false;
          bool vehicle_right = false;
          
          // Iterate through vehicles on the road and assess their position relative to Ego vehicle
          // Determine the State of the surrounding vehilces
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            // Get d value for tracked vehicle from sensor_fusion
            float d = sensor_fusion[i][6];
            
            // Define variable for the tracked vehicle lane
            int tracked_lane;
            
            // Check the d value for vehicle
            // return the lane number of the tracked vehicle
            if (d >= 0 && d < 4) {
              tracked_lane = 0;
            } else if (d >= 4 && d < 8) {
              tracked_lane = 1;
            } else if (d >= 8 && d <= 12) {
              tracked_lane = 2;
            } else {
              continue;}
            
            // Identify the driving state parameters of the tracked vehicle
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double tracked_vehicle_s = sensor_fusion[i][5];
              
            // Predict future s values in a time window
            // Variable lane_change_tune was tunned to avoid collisions during lane change
            int lane_change_tune = 0.03;
            tracked_vehicle_s += ((double)prev_size*lane_change_tune*check_speed);
              
            // Check if the s value is greater than Ego car and s gap
            // Set gap as a threshold after which action must be taken
            // Variable s_scale tunes how quickly the car changes lanes when passing a vehicle
            int gap = 20;
            int s_scale = 0.8;
            
            if((tracked_vehicle_s > ego_vehicle_s*s_scale) && ((tracked_vehicle_s - ego_vehicle_s) < gap))
            {
              // Check local vehilce state: ahead, to the left or right of the Ego vehicle
              if (tracked_lane == ego_lane) {
                // State: a vehicle is ahead of the ego vehicle and within the gap distance
                too_close |= (tracked_vehicle_s > ego_vehicle_s) && ((tracked_vehicle_s - ego_vehicle_s) < gap);
              } else if (tracked_lane - ego_lane == 1) {
                // State: a tracked vehicle is to the right of the ego vehicle
                vehicle_right |= ((ego_vehicle_s - gap) < tracked_vehicle_s) && ((ego_vehicle_s + gap) > tracked_vehicle_s);
              } else if (ego_lane - tracked_lane == 1) {
                // State: a tracked vehicle is to the left of the ego vehicle
                vehicle_left |= ((ego_vehicle_s - gap) < tracked_vehicle_s) && ((ego_vehicle_s + gap) > tracked_vehicle_s);
              }
            }
          }

          // Define variables for acceleration and maximum speed driving behavior
          double accel_inc = 0.25;
          double daccel_inc = 0.30;
          double max_speed = 49.5;
          
          // Defines lane change actions if vehicle is too close, the too_close flag is raised
          // Create lane change behavior rules
          // Decrease speed by an increment if too close, else, increase speed          
          // Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
          
          if (too_close) {
            // If a vehicle is ahead, gap threshold violated
            if (!vehicle_right && ego_lane < 2) {
              // If ego vehicle is in right two lanes
              // There is no trakced car to the right
              // Shift to the next lane (positive lane increment)
              ego_lane++;
            } else if (!vehicle_left && ego_lane > 0) {
              // If ego vehicle is one of the left two lanes
              // There is no tracked car to the left
              // Shift to the next lane (negative lane increment)
              ego_lane--;
            } else {
              // When no lanes are free, decrease speed, will be governed by the gap distance variable
              ref_vel -= daccel_inc;
            }
          } else {
            // If the above conditions do not apply
            // Lane ahead is free
            // Accelerate to the maximum speed
            if (ref_vel < max_speed) {
              ref_vel += accel_inc;
            }
          }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Create a list of widely spaced waypoints (x,y)
          // These are used to fit to a spline
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Reference (x,y) and yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // Check what the previous path size was
          if(prev_size < 2)
          {
          // Use two points that make the path tanget to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Redefine the reference state as the previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            //Use two points that make the path tanget to the previous path end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          // In Frenet space, add evenly spaced 30m points ahead of the starting reference
          // Lane width is 4m
          vector<double> next_wp0 = getXY(ego_vehicle_s+30, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(ego_vehicle_s+60, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(ego_vehicle_s+90, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++)
          {
          // Shift car reference angle to 0 degrees
          // Transformation to the local car coordinate system
          double shift_x = ptsx[i]-ref_x;
          double shift_y = ptsy[i]-ref_y;
            
          ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
          ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          
          // Create a spline
          tk::spline s;
          
          // Set (x,y) points to the spline curve
          s.set_points(ptsx,ptsy);
          
          // Define the actual (x,y) points to use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all of the pervious path points from last time state
          for(int i = 0; i < previous_path_x.size(); i++)
          {
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up the spline points
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          
          double x_add_on = 0;
          const int max_accel = 9;
          const double accel = (max_accel) * 0.02 * 0.8;
          
          // Fill up the rest of the path planner after filling it with the previous points
          // Always output 50 points
          for(int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to the normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          json msgJson;
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