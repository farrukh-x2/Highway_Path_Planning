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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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



std::tuple<double, double> transformWayPoints(double car_x, double car_y,double car_theta, 
		double wayp_x,  double wayp_y  ){
       //The way points and the car are given in the MAP'S coordinate system. 
       // Need to transform between the way points to car coordinate system.
       //This transformation requires FIRST translation and THEN rotation.

    double transf_x, transf_y;
    transf_x =     (wayp_x- car_x)*cos(car_theta) + (wayp_y- car_y)*sin(car_theta);
    transf_y =  -1*(wayp_x- car_x)*sin(car_theta) + (wayp_y- car_y)*cos(car_theta);

    return std::make_tuple(transf_x, transf_y);

}

template <class Sen_Fus>
bool check_car_in_lane(Sen_Fus sensor_fusion, int ref_lane, double main_car_last_s, int prev_size){

	for(int i = 0; i < sensor_fusion.size(); i++ )
	{	
	    double this_car_lane = sensor_fusion[i][6];

	    
	    if ((this_car_lane > ref_lane*4) && ( this_car_lane < (ref_lane+1)*4))
	    {

		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double this_car_Vmag = sqrt(vx*vx+ vy*vy);
		double this_car_s = sensor_fusion[i][5];

		double this_car_projected_s = this_car_s + ((double)prev_size*0.02*this_car_Vmag);

		if ((this_car_projected_s - main_car_last_s < 20) && (this_car_projected_s > main_car_last_s))
		{
		
		   return true;

		}

	    }

	}

    return false;

}

template <class Sen_Fus>
double dangerous_car_in_new_lane(Sen_Fus sensor_fusion, int lane_to_check, double main_car_last_s, int prev_size, double main_car_Vmag){
	double closest_car = 999.0;	
	for(int i = 0; i < sensor_fusion.size(); i++ )
	{	
	    double this_car_lane = sensor_fusion[i][6];

	    
	    if ((this_car_lane > lane_to_check*4) && ( this_car_lane < (lane_to_check+1)*4))
	    
	    {

		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double this_car_Vmag = sqrt(vx*vx+ vy*vy);
		double this_car_s = sensor_fusion[i][5];

		double this_car_projected_s = this_car_s + ((double)prev_size*0.02*this_car_Vmag);
		double this_car_projected_s_ten_time_units = this_car_s + ((double)prev_size*10*0.02*this_car_Vmag);

		if ( (fabs(main_car_last_s - this_car_projected_s ) < 15) )
		{

		   return 0;
		

		}
		
		if(this_car_projected_s_ten_time_units > main_car_last_s)
		{
		    
		    if ((this_car_projected_s_ten_time_units - main_car_last_s) < closest_car && 
			 (main_car_last_s - this_car_projected_s ) < 25) 
		    {

			closest_car = this_car_projected_s_ten_time_units - main_car_last_s;
		    }

		}

	    }

	}
    return closest_car;
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
	
  double ref_v = 0;//mph
  int ref_lane = 1;
  int last_lane_change = 0;
	
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_lane, &ref_v, &last_lane_change](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

		//double ref_v;
		int prev_size = previous_path_x.size();

		double main_car_last_s = car_s;	

		bool slow_down = false;	
		last_lane_change++;
		int time_interval = 2 * 50; // 50 iterations per seconds, so 2 seconds

 
		/////***********************************//////
		/////*******LANE CHANGE LOGIC - START***//////
		/////***********************************//////
		if (prev_size > 0)
		{
		    
		    main_car_last_s = end_path_s;
	
		}

		if (check_car_in_lane(sensor_fusion, ref_lane, main_car_last_s, prev_size))
		{
		    slow_down = true;
		    
		    // In the right most lane
		    if (ref_lane == (2) && last_lane_change > time_interval)
		    {

			if (dangerous_car_in_new_lane(sensor_fusion, 1, main_car_last_s, prev_size, car_speed) != 0 && ref_v < 45)
			  { 
			    ref_lane = 1;	
			    last_lane_change = 0;
			    cout<<"Change to middle from right"<<endl;
			  }			

		    }

		    // In the left most lane
		    if (ref_lane == (0) && last_lane_change > time_interval)
		    {

			if (dangerous_car_in_new_lane(sensor_fusion, 1, main_car_last_s, prev_size, car_speed) != 0 && ref_v < 45)
			  { 
			    ref_lane = 1;
			    last_lane_change = 0;	
			    cout<<"Change to middle from left"<<endl;
			  }			

		    }
		    // If in the middle lane
		    if (ref_lane == (1) && last_lane_change > time_interval )
		    {
			//Try to change to left lane
			double left_lane_closest_car = dangerous_car_in_new_lane(sensor_fusion, 0, main_car_last_s, prev_size, car_speed);
			double right_lane_closest_car = dangerous_car_in_new_lane(sensor_fusion, 2, main_car_last_s, prev_size, car_speed);

			cout<<"in Middle, left closest car:"<<left_lane_closest_car<<" right closest car: "<<right_lane_closest_car<<endl;
			
			if (left_lane_closest_car != 0 && left_lane_closest_car != 0 && ref_v < 45)
			{
			    if (left_lane_closest_car < right_lane_closest_car)
				{ref_lane =2;
				last_lane_change = 0;
			        cout<<"Can change to right from middle, closest_car: "<<right_lane_closest_car<<endl;}
			    else
				{ref_lane =0;
				last_lane_change = 0;
				cout<<"Can change to left from middle, closest_car: "<<left_lane_closest_car<<endl;}
			}
			else if (left_lane_closest_car != 0 && ref_v < 45)
			  { 
			    ref_lane = 0;	
			    last_lane_change = 0;	
			    cout<<"Can change to left from middle, closest_car: "<<left_lane_closest_car<<endl;
			  }
			
			else if (right_lane_closest_car != 0 && ref_v < 45)
			  { 
			    ref_lane = 2;	
			    last_lane_change = 0;	
			    cout<<"Can change to right from middle, closest_car: "<<right_lane_closest_car<<endl;
			  }
			
		    }

		   
		}
		/////***********************************//////
		/////*******LANE CHANGE LOGIC - END***//////
		/////***********************************//////


		if (slow_down || ref_v > 48.0 )
		{
		ref_v -= .55; 
		}
		else if (ref_v < 48.0)
		{
		ref_v += .75;
		}

		
	
		/////**************//////
		// Create vectors to store a list of few points to generate a spline. Further spline points will be generated just before passing the final points to the simulator
		vector<double> ptsx;
		vector<double> ptsy;

		// The reference x,y and yaw states
		// This is either where the car is or the last point from previous path
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		double prev_ref_x, prev_ref_y;


		if (prev_size < 2)
		{
		   // Since we don't have the previous points we calculate the last point from the current car angle and going back one unit
		   prev_ref_x = car_x - 1*cos(car_yaw);
		   prev_ref_y = car_y - 1*sin(car_yaw);

		   ptsx.push_back(prev_ref_x);
		   ptsx.push_back(ref_x);

		   ptsy.push_back(prev_ref_y);
		   ptsy.push_back(ref_y);


		}

		else
		{  // Use previous two points to store in the waypoints for spline list
		   ref_x = previous_path_x[prev_size - 1];
		   prev_ref_x = previous_path_x[prev_size - 2];
		
		   ref_y = previous_path_y[prev_size - 1];
   		   prev_ref_y = previous_path_y[prev_size - 2];

 		   car_yaw = atan2((ref_y - prev_ref_x),(ref_x - prev_ref_x));

		   ptsx.push_back(prev_ref_x);
		   ptsx.push_back(ref_x);

		   ptsy.push_back(prev_ref_y);
		   ptsy.push_back(ref_y);	

		}		
  		

		// We want to assign 3 more points to the waypoints for spline list
		vector<double> WP1 = getXY(car_s + 50, (ref_lane*4+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> WP2 = getXY(car_s + 65, (ref_lane*4+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> WP3 = getXY(car_s + 90, (ref_lane*4+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		
		ptsx.push_back(WP1[0]);
		ptsx.push_back(WP2[0]);
		ptsx.push_back(WP3[0]);

		ptsy.push_back(WP1[1]);
		ptsy.push_back(WP2[1]);
		ptsy.push_back(WP3[1]);

		
		
		// transform to coordinates relative to the car instead of the map. This makes it easier to work with

		for (int i = 0; i < ptsx.size(); i++)
		{

		    tie(ptsx[i],ptsy[i]) = transformWayPoints(ref_x, ref_y, ref_yaw, ptsx[i], ptsy[i]);
		
		}


		// the points that we will pass to the simulator
	       	vector<double> next_x_vals;
		vector<double> next_y_vals;

		//First we will fill in with all the previous points so the transistion is very smooth
		for (int i =0; i < previous_path_x.size(); i++ )
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		
		}


		// make a new spline
		tk::spline s;

		// set it
		s.set_points(ptsx, ptsy);

		//we need to split the spline into equal interval based on the required speed. If the interval aren't uniform and are not far apart enough there will be accerelation jerks
		double final_x = 30.0;
		double final_y = s(final_x);
		double shortest_dis = sqrt((final_x*final_x) + (final_y*final_y));


		double base_x = 0;
		// for the points in the new path that are not there	
		for (int i = 1; i < 50 - previous_path_x.size(); i ++)
		{	
   		    double N = (shortest_dis / (0.02*ref_v/2.24));	
		    double temp_x = base_x + (final_x/N);
		    double temp_y = s(temp_x);
		    //cout<<"*****final "<<temp_x<<" "<< temp_y<<endl;		    
		    base_x = temp_x;

		    double x_ref = temp_x;
		    double y_ref = temp_y;

		    //Transform back to map coordinates			
		    temp_x = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
		    temp_y = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
		    
		    // add to current position of car or the last position in the previous path, as 
		    // the path continues from there.
		    temp_x += ref_x;
		    temp_y += ref_y;

		    next_x_vals.push_back(temp_x);
		    next_y_vals.push_back(temp_y);

		    //cout<<"final "<<temp_x<<" "<< temp_y<<endl;		    
		    
			

		} 
		

		msgJson["next_x"] = next_x_vals;
		msgJson["next_y"] = next_y_vals;


    		/////**************//////


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

