#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

bool CheckLeftLane(int lane, vector<vector<double> > car_track, vector<vector<double> > sensor_fusion, int path_size, double car_s){
	
	int lane_to_check = lane - 1;
	
	for(int i = 0 ; i < car_track[lane_to_check].size(); i ++){ //loop for 0,1 and 2 lane numbers
	
		double vx = sensor_fusion[car_track[lane_to_check][i]][3];
		double vy = sensor_fusion[car_track[lane_to_check][i]][4];
		double check_speed = sqrt((vx*vx)+(vy*vy));
		double check_car_s = sensor_fusion[car_track[lane_to_check][i]][5];
		
		check_car_s += ((double)path_size*0.02*check_speed); //if using previous points can project s value out
		//check if the car is in front of us and also if the gap is less than 30 or not
		
		if((check_car_s < car_s) && ((car_s - check_car_s) < 10)){
			return false;
		}
		if( (check_car_s > car_s) && ((check_car_s - car_s) < 30)){
			return false;
		}
		
	}
	
	return true;
}

bool CheckRightLane(int lane, vector<vector<double> > car_track, vector<vector<double> > sensor_fusion, int path_size, double car_s){
	
	int lane_to_check = lane + 1;
	
	for(int i = 0 ; i < car_track[lane_to_check].size(); i ++){ //loop for 0,1 and 2 lane numbers
	
		double vx = sensor_fusion[car_track[lane_to_check][i]][3];
		double vy = sensor_fusion[car_track[lane_to_check][i]][4];
		double check_speed = sqrt((vx*vx)+(vy*vy));
		double check_car_s = sensor_fusion[car_track[lane_to_check][i]][5];
		
		check_car_s += ((double)path_size*0.02*check_speed); //if using previous points can project s value out
		//check if the car is in front of us and also if the gap is less than 30 or not
		
		if((check_car_s < car_s) && ((car_s - check_car_s) < 10)){
			return false;
		}
		if( (check_car_s > car_s) && ((check_car_s - car_s) < 30)){
			return false;
		}
		
	}
	
	return true;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  int lane = 1;
  double ref_vel = 2;
  
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			bool print_statements = false;
			int path_size = previous_path_x.size();
			
			cout<<endl;
		
			if(path_size > 0){
				car_s = end_path_s;
			}
			
			bool too_close = false;
			
			int number_of_lanes = 3; //0,1 and 2   we are concerned only with 1,2 and 3 
			vector<vector<double> > car_track(number_of_lanes); 
			
			for(int i = 0; i < sensor_fusion.size(); i ++){
				float d = sensor_fusion[i][6];
				if(d >= 8){ //lane 3 car
					//car_track[2].push_back(sensor_fusion[i][0]);
					car_track[2].push_back(i);					
					if (print_statements){
						cout<<"d>8 : "<<d<<endl;
					}
				}else if (d >= 4){ //lane 2 car
					//car_track[1].push_back(sensor_fusion[i][0]);
					car_track[1].push_back(i);
					if (print_statements){
					cout<<"d>4 : "<<d<<endl;
					}
				}else if (d>0){//left most lane
					//car_track[0].push_back(sensor_fusion[i][0]);
					car_track[0].push_back(i);
					if (print_statements){
					cout<<"Else: "<<d<<endl;
					}
				}						
			}
			
			if (print_statements){
				if(ref_vel < 8){
					cout<<"Sensor fusion size : "<<sensor_fusion.size()<<endl;
					cout<<"car_track[1].size = "<<car_track[0].size()<<endl;
					cout<<"car_track[2].size = "<<car_track[1].size()<<endl;
					cout<<"car_track[3].size = "<<car_track[2].size();
					cout<<endl<<endl;
				}
			}
			
			for(int i = 0 ; i < car_track[lane].size(); i ++){ //loop for 0,1 and 2 lane numbers
			
				double vx = sensor_fusion[car_track[lane][i]][3];
				double vy = sensor_fusion[car_track[lane][i]][4];
				double check_speed = sqrt((vx*vx)+(vy*vy));
				double check_car_s = sensor_fusion[car_track[lane][i]][5];
				
				check_car_s += ((double)path_size*0.02*check_speed); //if using previous points can project s value out
				//check if the car is in front of us and also if the gap is less than 30 or not
				if((check_car_s > car_s ) && ((check_car_s - car_s) < 30)){
					//ref_vel = 29;
					too_close = true;
					/*if(lane > 0){
						lane = 0;
					}*/
					break;
				}
			}
			
			if(too_close){	
				switch(lane){
					
					case 0:
						if (CheckRightLane(lane,car_track,sensor_fusion,path_size, car_s)){
							lane = 1;
						}
						break;
					case 1:
						if (CheckLeftLane(lane,car_track,sensor_fusion, path_size, car_s)){
							lane = 0;
						}else if (CheckRightLane(lane,car_track,sensor_fusion, path_size, car_s)){
							lane = 2;
						}
						break;
					case 2:
						if (CheckLeftLane(lane,car_track,sensor_fusion, path_size, car_s)){
							lane = 1;
						}
						break;
					default:
						//Do nothing
						;
				}
			}
			
			//These will contain spaced points that will be passed to spline to get a trajectory
			vector<double> x_points_for_spline;
			vector<double> y_points_for_spline;
			
			//Creating reference ponits
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			
			
			if(path_size<2){
				//Path is almost empty so create our own points using equations
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);
				
				x_points_for_spline.push_back(prev_car_x);
				x_points_for_spline.push_back(car_x);
				
				y_points_for_spline.push_back(prev_car_y);
				y_points_for_spline.push_back(car_y);
				
			}else{
				//if we have previous points then use them
				ref_x = previous_path_x[path_size-1];
				ref_y = previous_path_y[path_size-1];
				
				double ref_prev_car_x = previous_path_x[path_size-2];
				double ref_prev_car_y = previous_path_y[path_size-2];
				
				ref_yaw = atan2(ref_y- ref_prev_car_y, ref_x-ref_prev_car_x);
				
				x_points_for_spline.push_back(ref_prev_car_x);
				x_points_for_spline.push_back(ref_x);
				
				y_points_for_spline.push_back(ref_prev_car_y);
				y_points_for_spline.push_back(ref_y);				
				
				
			}
			
			
			
			//Using 5 points in Frenet system for spline. Since we now have 2 starting points now getting the rest points.
			vector<double> spline_point3 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> spline_point4 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> spline_point5 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			x_points_for_spline.push_back(spline_point3[0]);
			x_points_for_spline.push_back(spline_point4[0]);
			x_points_for_spline.push_back(spline_point5[0]);

			y_points_for_spline.push_back(spline_point3[1]);
			y_points_for_spline.push_back(spline_point4[1]);
			y_points_for_spline.push_back(spline_point5[1]);
			
			
			//or use path_size
			for(int i = 0; i < x_points_for_spline.size(); i++){
				//shifting to car co-ordinate system
				double shift_x = x_points_for_spline[i] - ref_x;
				double shift_y = y_points_for_spline[i] - ref_y;
				
				x_points_for_spline[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
				y_points_for_spline[i] = (shift_y*cos(0-ref_yaw) + shift_x*sin(0-ref_yaw));
			}
			
			tk::spline s;
			
			s.set_points(x_points_for_spline, y_points_for_spline);
			
			double dist_inc = 0.4;
			
			//or use path_size
			for(int i = 0; i < previous_path_x.size(); i++){
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}
			
			//Break the spline points so that we do not overshoot the reference velocity
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_distance = sqrt((target_x*target_x)+(target_y*target_y));
			
			double x_add_on = 0;
			
			//or use already computer path_size
			for(int i = 0; i < 50 - previous_path_x.size(); i++){
				//0.224 -> this comes when we take acceleration as 5meter/second^2
				if(too_close){
					ref_vel = ref_vel - 0.224;
				}else if (ref_vel < 49){
					ref_vel = ref_vel + 0.224;
				}
				//divide by 2.24 because this equation is for m/s and not Miles Per Hour. ref_vel is set in Miles Per Hour
				double N = (target_distance/(0.02*ref_vel/2.24));
				double x_point = x_add_on + target_x/N;
				double y_point = s(x_point);
				
				x_add_on = x_point;
				
				
				//transform back to global co-ordinate system
				double x_ref = x_point;
				double y_ref = y_point;
				
				x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (y_ref*cos(ref_yaw)+x_ref*sin(ref_yaw));
				
				x_point += ref_x;
				y_point += ref_y;
				
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}
			
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
