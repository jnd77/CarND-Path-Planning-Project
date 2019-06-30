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

// Function to check if the target lane is safe to move to, and it's worth moving to
bool check_target_lane(int lane, double car_s, double delta_t, vector<vector<double>> sensor_fusion) {
  bool change_lane = true;

  for(int i = 0; i < sensor_fusion.size(); i++) {
	float d = sensor_fusion[i][6];

	// If car is in same lane
	if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double other_car_speed = sqrt(vx * vx + vy * vy);
		double other_car_s = sensor_fusion[i][5];

		// Since we will compare with car_s (the last s, we should predict where the car will be at that time)
		double other_car_future_s = other_car_s + delta_t * other_car_speed;

		// if car is too close behind
		if (other_car_future_s < car_s && (car_s - other_car_future_s) < 5) {
			change_lane = false;
		}

		// if car is too close in front
		if (other_car_future_s >= car_s && (other_car_future_s - car_s) < 30) {
			change_lane = false;
		}
	}
  }

  return change_lane;
}

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


  // Initial values
  int lane = 1;
  double ref_speed = 0;  // in m/s
  bool changing_lane = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_speed, &changing_lane]
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  int num_prev_points = previous_path_x.size();

		  // Have we finished changing lane
		  if (changing_lane && abs(car_d - (2 + lane * 4)) < 0.1)
		    changing_lane = false;

		  /* 
		  * Decide what action to do: stay in same lane, or change
		  */
		  // We check if car is too close
		  if (num_prev_points > 0)
		    car_s = end_path_s;

		  bool too_close = false;
		  bool slow_down = false;

		  // Loop over all the detected cars
		  double delta_t = ((double) num_prev_points) * 0.02;
		  for(int i = 0; i < sensor_fusion.size(); i++) {
		  	  float d = sensor_fusion[i][6];

			  // If car is in "same lane"
			  if (fabs(d - car_d) < 3) {
			  	  double vx = sensor_fusion[i][3];
				  double vy = sensor_fusion[i][4];
				  double other_car_speed = sqrt(vx * vx + vy * vy);
				  double other_car_s = sensor_fusion[i][5];

				  // Since we will compare with car_s (the last s, we should predict where the car will be at that time)
				  double other_car_future_s = other_car_s + delta_t * other_car_speed;

				  // If car is in front and less than 30m, then we aree too close
				  if (other_car_future_s > car_s && (other_car_future_s - car_s) < 30) {
				  	  too_close = true;
					  if (car_speed > other_car_speed + 2)
					    slow_down = true;
				  }
			  }
		  }

		  if (too_close) {
		    // Slow down
			if (slow_down)
			  ref_speed -= 6 * 0.02;	// Max acceleration is 10 m.s-2 (so use 6 to stay under)
			
			// If not in process of changing lanes, then try
			if (! changing_lane) {
			  // Check if other lanes are possible and better
			  int target_lane = lane;
			  // Go left (if possible)
			  if (lane > 0 && check_target_lane(lane - 1, car_s, delta_t, sensor_fusion))
				target_lane = lane - 1;
			  
			  // Go right (if possible and left lane was not possible)
			  if (lane < 2 && target_lane == lane && check_target_lane(lane + 1, car_s, delta_t, sensor_fusion))
				target_lane = lane + 1;

			  changing_lane = (target_lane != lane);
			  lane = target_lane;
			}
		  } else if (ref_speed < 22.1) {  // 22.1 m/s is slightly less than 50 mile/hour
			  ref_speed += 6 * 0.02;
		  }

		  /* 
		  * Then generate the appropriate trajectory
		  */
		  // Create list of far-spaced trajectory points (smooth detailed trajectory will be generated from these points)
		  vector<double> points_x;
		  vector<double> points_y;

		  // Keep track of reference state
		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);

		  // For smooth trajectories, good idea to use previous trajectory points
		  if (num_prev_points < 2) {
		  	  // Almost no points, so might as well use car position to start (estimated previous point plus current point)
			  points_x.push_back(car_x - cos(car_yaw));
			  points_y.push_back(car_y - sin(car_yaw));

			  points_x.push_back(car_x);
			  points_y.push_back(car_y);
		  } else {
		  	  // Otherwise use previous path points

			  // First overwrite the ref position with the last previous point
			  ref_x = previous_path_x[num_prev_points - 1];
			  ref_y = previous_path_y[num_prev_points - 1];

			  // Get previous point as well
			  double ref_x_prev = previous_path_x[num_prev_points - 2];
			  double ref_y_prev = previous_path_y[num_prev_points - 2];
			  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			  // Add both to the trajectory points
			  points_x.push_back(ref_x_prev);
			  points_y.push_back(ref_y_prev);

			  points_x.push_back(ref_x);
			  points_y.push_back(ref_y);
		  }
		  
		  // Add 3 more points in 30, 60 and 90 meters (in same lane, so better use Frenet)
		  for(int i = 0; i < 3; i++) {
		  	  vector<double> next_point = getXY(car_s + (i + 1) * 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			  points_x.push_back(next_point[0]);
			  points_y.push_back(next_point[1]);
		  }

		  // Change the points from map to vehicle coordinates
		  for(int i = 0; i < points_x.size(); i++) {
		  	  double shift_x = points_x[i] - ref_x;
			  double shift_y = points_y[i] - ref_y;

			  points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
			  points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
		  }
		  
		  // Generate the trajectory via spline through the points
		  tk::spline s;
		  s.set_points(points_x, points_y);

		  vector<double> next_x_vals;
          vector<double> next_y_vals;

		  // Reuse all past points
		  for(int i = 0; i < num_prev_points; i++) {
		    next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		  }

		  // Split the spline in small segments
		  double target_x = 30;
		  double target_dist = distance(0, 0, target_x, s(target_x));
		  double num_segments = target_dist / (0.02 * ref_speed);
		  double x_add_on = 0;

		  for(int i = 1; i < 50 - num_prev_points; i++) {
		    // Compute car coordinates
		    double x_point = x_add_on + target_x / num_segments;
			double y_point = s(x_point);
		  
		    // Update the x
		    x_add_on = x_point;

			// Switch back to map coordinates
			double x_ref = x_point;
			double y_ref = y_point;

			x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
			y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

			x_point += ref_x;
			y_point += ref_y;

			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);
		  }

		  // Create the Json answer
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
