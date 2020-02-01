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

#define max_speed_kmph 79
#define max_speed_mps  22
#define max_lane_change_mps 16
#define max_acc_mpss    9

#define data_duration 0.02
#define speed_increment_kmph 1.58
#define speed_increment_mps 0.16
#define acc_increment_mpss 0.18

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum states {
    READY,
    KEEP_LANE, // Stay at same lane, keep d constant, s keep current speed or try to match current lane speed
    LANE_CHANGE_LEFT, // Change d for left lane (-4), if possible, s keep current speed or try to match current lane speed
    LANE_CHANGE_RIGHT, // change d for right lane (+4), if possible, s keep current speed or try to match current lane speed
    PREPARE_LANE_CHANGE_LEFT,// try to kep d same, check for gap in nearby lanes and try to match their speed
    PREPARE_LANE_CHANGE_RIGHT,
} sm_state;

enum events {
    KEEP_LANE_EVENT,
    GOTO_KEEP_LANE_EVENT,
    LANE_CHANGE_LEFT_EVENT,
    LANE_CHANGE_RIGHT_EVENT,
    PREPARE_LANE_CHANGE_LEFT_EVENT, // trigger this event when we are ~ 30m behind of car in front of us and trigger lane change indicator as this is part of BP
    PREPARE_LANE_CHANGE_RIGHT_EVENT,
};

typedef bool(*state_handler)(events event, string s);

typedef struct {
	states current_state;
	state_handler handle_event;
} state_machine;

struct ego_car_params {
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
	
	vector<double> previous_path_x;
	vector<double> previous_path_y;

	double end_path_s;
	double end_path_d;
};

struct traffic_car_param {
	int car_id;
	double car_pos_x;
	double car_pos_y;
	double car_vel_x;
	double car_vel_y;
	double car_pos_s;
	double car_pos_d;
};

state_machine sm;
vector<double> nearest_car {100000,100000,100000,100000,100000,100000}; // front and back for each lane 
vector<double> nearest_car_speed {0.0,0.0,0.0,0.0,0.0,0.0} ;// front and back for each lane 
vector<double> lane_costs {100000.0,100000.0,100000.0}; // set costs for each lane to default values.

double current_vel = 0.0;

bool ready_handler(events event, string s);
bool keep_lane_handler(events event, string s);
bool lane_change_left_handler(events event, string s);
bool lane_change_right_handler(events event, string s);
bool prepare_lane_change_left_handler(events event, string s);
bool prepare_lane_change_right_handler(events event, string s);
ego_car_params get_ego_parmas(string s);
int get_lane_from_d (double d);
int get_lane_middle (double d);
void reset_nearest_data();
void fill_nearest_data(string s);
void fill_lane_costs(int ego_lane, bool left_lane, bool current_lane, bool right_lane);
bool is_lane_safe(int ego_lane, int target_lane, double ego_speed);
double get_traffic_future_s(int index);
double get_safe_velocity(int ego_lane, bool left_lane, bool current_lane, bool right_lane);
int get_prev_list_size(vector<double> previous_path);
vector<traffic_car_param> get_traffic_params(string s);
vector<double> get_lane_change_d(int ego_lane, int target_lane);


const state_handler event_handlers [] = {
	ready_handler,
	keep_lane_handler,
	lane_change_left_handler,
	lane_change_right_handler,
	prepare_lane_change_left_handler,
	prepare_lane_change_right_handler
};

ego_car_params get_ego_parmas(string s) {

	ego_car_params car_params;
	int i = 0;
	if (s != "") { 
		auto j = json::parse(s);	
		string event = j[0].get<string>();	
		if (event == "telemetry") {
			auto temp = j[1]["previous_path_x"];
			i = temp.size();
			car_params.car_x = j[1]["x"];
			car_params.car_y = j[1]["y"];
			car_params.car_s = j[1]["s"];
			car_params.car_d = j[1]["d"];
			car_params.car_yaw = j[1]["yaw"];
			car_params.car_speed = j[1]["speed"];
			for (int k = 0; k < i; k++) {
				car_params.previous_path_x.push_back(j[1]["previous_path_x"][k]);
				car_params.previous_path_y.push_back(j[1]["previous_path_y"][k]);
			}
			car_params.end_path_s = j[1]["end_path_s"];
			car_params.end_path_d = j[1]["end_path_d"];
		}
	}

	return car_params;
}
vector<traffic_car_param> get_traffic_params(string s, int traffic_size) {

	vector<traffic_car_param> traffic_params;
	traffic_car_param temp_params;

	if (s != "") { 
		auto j = json::parse(s);	
		string event = j[0].get<string>();	
		if (event == "telemetry") {
			auto sensor_fusion = j[1]["sensor_fusion"];
			//std::cout<< "traffic cars count " << traffic_size << std::endl;
			for (int i = 0; i < traffic_size; i++) {
				//std::cout<< "traffic s " << sensor_fusion[i][5] << " tarffic d " << sensor_fusion[i][6]<< std::endl;
				temp_params.car_id = sensor_fusion[i][0];
				temp_params.car_pos_x =sensor_fusion[i][1];
				temp_params.car_pos_y =sensor_fusion[i][2];
				temp_params.car_vel_x =sensor_fusion[i][3];
				temp_params.car_vel_y =sensor_fusion[i][4];
				temp_params.car_pos_s =sensor_fusion[i][5];
				temp_params.car_pos_d =sensor_fusion[i][6];
				traffic_params.push_back(temp_params);
			}
		}
	}
	return traffic_params;
}

int get_traffic_size (string s) {
	int traffic_size = 0;
	if (s != "") { 
		auto j = json::parse(s);	
		string event = j[0].get<string>();	
		if (event == "telemetry") {
			auto sensor_fusion = j[1]["sensor_fusion"];
			if (sensor_fusion != "")
				traffic_size = sensor_fusion.size();
		}
	}
	return traffic_size;
}

int get_lane_from_d (double d) {

	if (d > 0 && d <=4 )
		return 0;
	else if (d > 4 && d <= 8)
		return 1;
	else
		return 2;

}

int get_lane_middle (double d) {

	return get_lane_from_d(d) * 4 +2;

}

void reset_nearest_data () {

	for (int i = 0; i < nearest_car.size(); i++) {
		nearest_car[i] = 100000;
		nearest_car_speed[i] = 0.0;
	}
}
void fill_nearest_data(string s) {

	ego_car_params car_params = get_ego_parmas(s);
	int traffic_size = get_traffic_size(s);
	int ego_lane =	get_lane_from_d(car_params.car_d);
	
	vector<traffic_car_param> traffic_car = get_traffic_params(s,traffic_size);
	for (int i = 0; i < traffic_size; i++) {
	  int traffic_car_lane = get_lane_from_d(traffic_car[i].car_pos_d);
	  double diff = traffic_car[i].car_pos_s - car_params.car_s;
	  double traffic_speed = sqrt(traffic_car[i].car_vel_x * traffic_car[i].car_vel_x +
	  		traffic_car[i].car_vel_y * traffic_car[i].car_vel_y);
	  //std::cout<<"traffic_car_lane " << traffic_car_lane << " diff "<< diff <<	std::endl;
	
	  if (traffic_car_lane == 0) {
		  if (diff > 0) {
			  if (diff < nearest_car[0]) {
				  nearest_car[0] = diff;
				  nearest_car_speed[0] = traffic_speed;
			  }
		  } else {
			  if (std::abs(diff) < nearest_car[1]) {
				  nearest_car[1] = std::abs(diff);
				  nearest_car_speed[1] = traffic_speed;
			  }
		  }
	  } else if (traffic_car_lane == 1) {
		  if (diff > 0) {
			  if (diff < nearest_car[2]) {
				  nearest_car[2] = diff;
				  nearest_car_speed[2] = traffic_speed;
			  	}
		  } else {
			  if (std::abs(diff) < nearest_car[3]) {
				  nearest_car[3] = std::abs(diff);
				  nearest_car_speed[3] = traffic_speed;
			  }
		  }
	  } else if (traffic_car_lane ==2) {
		  if (diff > 0) {
			  if (diff < nearest_car[4]) {
				  nearest_car[4] = diff;
				  nearest_car_speed[4] = traffic_speed;
			  }
		  } else {
			  if (std::abs(diff) < nearest_car[5]) {
				  nearest_car[5] = std::abs(diff);
				  nearest_car_speed[5] = traffic_speed;
			  }
		  }
	  } else {
		  std::cout << "invalid lane traffic " << traffic_car_lane << std::endl;
	  }
	}


}

void fill_lane_costs(int ego_lane, bool left_lane, bool current_lane, bool right_lane) {
	//lane_costs

	// if both left and right lane are safe swicth to lane which has 
	// car farthest from ego car. if there are no cars on left and
	// right lane then preffer right lane as it farthest from opposite 
	// traffic.

	// overtake a car only if speed of front car is ~75% of our speed
	// while overtake always preffer to take left and switch back to
	// old lane

	// 
	if (right_lane) {
		lane_costs = {1000.0,1000.0, 100};
	}
	
	double left_lane_s = nearest_car[0] + nearest_car[1];
	double center_lane_s = nearest_car[2] + nearest_car[3];
	double right_lane_s = nearest_car[4] + nearest_car[5];

	double left_lane_time = left_lane_s / current_vel;
	double center_lane_time = center_lane_s / current_vel;	
	double right_lane_time = center_lane_s / current_vel;

	
	if (left_lane && current_lane && right_lane) {
		
	}
	
}


double get_traffic_future_s(int index) {

	double future_s = 0.02 * nearest_car_speed[index] + nearest_car[index];
	return future_s;
}

bool is_lane_safe(int ego_lane, int target_lane, double ego_speed) {

	double front_s = 0.0;
	double back_s = 0.0;

	if (target_lane < 0 || target_lane > 2) {
		// no left on left lane and no right on right lane
		return false;
	} else if (ego_lane == target_lane ) {
		// check for current lane
		front_s = get_traffic_future_s((ego_lane * 2)) - ego_speed * 0.02;
		back_s = get_traffic_future_s((ego_lane * 2 + 1)) - ego_speed * 0.02;

	} else {
		front_s = get_traffic_future_s((target_lane * 2));
		back_s = get_traffic_future_s((target_lane * 2 + 1));
	}

	//std::cout << "target_lane " << target_lane << " front_s " << front_s << " back_s " << back_s << std::endl;	
	if (front_s > 40 && back_s > 10) // tune this param once we reuse last calculated points
		return true;
	else 
		return false;
}

int get_prev_list_size(vector<double> previous_path) {
	return previous_path.size();
}


bool ready_handler(events event, string s) {
	std::cout<<"ready_handler" << std::endl;
	ego_car_params ego = get_ego_parmas(s);
	int traffic_size = get_traffic_size(s);
	vector<traffic_car_param> traffic;
	traffic = get_traffic_params(s, traffic_size);
}

bool keep_lane_handler(events event, string s) {
	std::cout<<"keep_lane_handler" << std::endl;;

	ego_car_params ego = get_ego_parmas(s);

	int traffic_size = get_traffic_size(s);
	vector<traffic_car_param> traffic;
	if (traffic_size != 0)
		traffic = get_traffic_params(s, traffic_size);

	switch (event)
	{
		case KEEP_LANE_EVENT:
			break;
		case GOTO_KEEP_LANE_EVENT:
			break;
		case LANE_CHANGE_LEFT_EVENT:
			break;
		case LANE_CHANGE_RIGHT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_LEFT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_RIGHT_EVENT:
			break;
		default:
			break;
	}
	
}

bool lane_change_left_handler(events event, string s) {
	std::cout<<"lane_change_left_handler" << std::endl;;

	ego_car_params ego = get_ego_parmas(s);

	int traffic_size = get_traffic_size(s);
	vector<traffic_car_param> traffic;
	if (traffic_size != 0)
		traffic = get_traffic_params(s, traffic_size);

	switch (event)
	{
		case KEEP_LANE_EVENT:
			break;
		case GOTO_KEEP_LANE_EVENT:
			break;
		case LANE_CHANGE_LEFT_EVENT:
			break;
		case LANE_CHANGE_RIGHT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_LEFT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_RIGHT_EVENT:
			break;
		default:
			break;
	}

}

bool lane_change_right_handler(events event, string s) {

	std::cout<<"lane_change_right_handler" << std::endl;;

	ego_car_params ego = get_ego_parmas(s);

	int traffic_size = get_traffic_size(s);
	vector<traffic_car_param> traffic;
	if (traffic_size != 0)
		traffic = get_traffic_params(s, traffic_size);

	switch (event)
	{
		case KEEP_LANE_EVENT:
			break;
		case GOTO_KEEP_LANE_EVENT:
			break;
		case LANE_CHANGE_LEFT_EVENT:
			break;
		case LANE_CHANGE_RIGHT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_LEFT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_RIGHT_EVENT:
			break;
		default:
			break;
	}

}

bool prepare_lane_change_left_handler(events event, string s) {
	std::cout<<"prepare_lane_change_left_handler" << std::endl;;

	ego_car_params ego = get_ego_parmas(s);

	int traffic_size = get_traffic_size(s);
	vector<traffic_car_param> traffic;
	if (traffic_size != 0)
		traffic = get_traffic_params(s, traffic_size);

	switch (event)
	{
		case KEEP_LANE_EVENT:
			break;
		case GOTO_KEEP_LANE_EVENT:
			break;
		case LANE_CHANGE_LEFT_EVENT:
			break;
		case LANE_CHANGE_RIGHT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_LEFT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_RIGHT_EVENT:
			break;
		default:
			break;
	}

}

bool prepare_lane_change_right_handler(events event, string s) {

	std::cout<<"prepare_lane_change_right_handler" << std::endl;;

	ego_car_params ego = get_ego_parmas(s);

	int traffic_size = get_traffic_size(s);
	vector<traffic_car_param> traffic;
	if (traffic_size != 0)
		traffic = get_traffic_params(s, traffic_size);

	switch (event)
	{
		case KEEP_LANE_EVENT:
			break;
		case GOTO_KEEP_LANE_EVENT:
			break;
		case LANE_CHANGE_LEFT_EVENT:
			break;
		case LANE_CHANGE_RIGHT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_LEFT_EVENT:
			break;
		case PREPARE_LANE_CHANGE_RIGHT_EVENT:
			break;
		default:
			break;
	}

}

vector<double> get_lane_change_d(int ego_lane, int target_lane) {

	vector<double> d;
	int edo_d = ego_lane * 4 + 2;
	for (int i = 0 ; i < 8; i++) {
		d.push_back(edo_d + 0.5*target_lane*(i+1));
		//std::cout << " " << d[i] << " ";
	}
	std::cout<< std::endl;
	return d;
}

double get_safe_velocity(int ego_lane, bool left_lane, bool current_lane, bool right_lane) {

	if (current_vel <= (max_speed_mps - speed_increment_mps) && current_lane) {
		current_vel += speed_increment_mps;
	} else if (!current_lane) {
		if (current_vel > nearest_car_speed[2*ego_lane])
			current_vel -= speed_increment_mps;
	} else if (!left_lane && !right_lane) {
      std::cout<< "left and right lane is not safe" << std::endl;
	}
    return current_vel;
}

int main() {

  sm.current_state = READY;
  sm.handle_event = (const state_handler)event_handlers;

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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           // change this states from event handler rather from here
		  //sm.current_state = KEEP_LANE;
		  //lane_change_left_handler(KEEP_LANE_EVENT, s);

		  reset_nearest_data();
		  fill_nearest_data(s);

		  ego_car_params car_params = get_ego_parmas(s);
  		  int ego_lane = get_lane_from_d(car_params.car_d);
		  int lane = ego_lane;
		  int target_lane = 0;
		  bool is_left_lane_safe = is_lane_safe(ego_lane, ego_lane - 1, car_params.car_speed);
		  bool is_current_safe = is_lane_safe(ego_lane,ego_lane, car_params.car_speed);
		  bool is_right_lane_safe = is_lane_safe(ego_lane,ego_lane + 1, car_params.car_speed);

		  fill_lane_costs(ego_lane, is_left_lane_safe, is_current_safe, is_right_lane_safe);
		  int prev_list_size = get_prev_list_size(car_params.previous_path_x);
		  
		  //bool is_current_safe = is_lane_safe(ego_lane,3, car_params.car_speed); // special case for checking current lane status
		  //vector<double> next_wp0, next_wp1, next_wp2, next_wp3, next_wp4, next_wp5;
		  //vector<double> next_wp6, next_wp7, next_wp8, next_wp9, next_wp10, next_wp11;

		  //std::cout << "prev_list_size " << prev_list_size << std::endl;
		  std::cout << is_left_lane_safe << "  " << is_current_safe << " " << is_right_lane_safe << std::endl;
		  for (int i = 0; i < prev_list_size ; i++) {
				next_x_vals.push_back(car_params.previous_path_x[i]);
				next_y_vals.push_back(car_params.previous_path_y[i]);
        	}

		  if (!is_current_safe) {
			//  std::cout << "initiate lane change from " << ego_lane << std::endl;
			  if (is_left_lane_safe) {
			  	//std::cout << "can move to left "<< std::endl;
				target_lane = -1;
   			    //current_vel -= speed_increment_mps;
			  }
			  if (is_right_lane_safe) {
			  	//std::cout << "can move to right "<< std::endl;
				target_lane = 1;
			    //current_vel -= speed_increment_mps;
			  }
			  if (is_right_lane_safe && is_left_lane_safe) {
			  	//std::cout << "can move to both sides "<< std::endl;
			  }
		  }

		  vector<double> point_x;
		  vector<double> point_y;
		  double ego_last_x = car_params.car_x;
		  double ego_last_y = car_params.car_y;
		  double current_yaw = deg2rad(car_params.car_yaw);
		  
		  if (prev_list_size < 2) {

			point_x.push_back(car_params.car_x - cos(car_params.car_yaw));
			point_x.push_back(car_params.car_x);
		  
		  	point_y.push_back(car_params.car_y - sin(car_params.car_yaw));
			point_y.push_back(car_params.car_y);
		  } else {
			ego_last_x = car_params.previous_path_x[prev_list_size - 1];
			ego_last_y = car_params.previous_path_y[prev_list_size - 1];

			double ego_last_last_x = car_params.previous_path_x[prev_list_size - 2];
			double ego_last_last_y = car_params.previous_path_y[prev_list_size - 2];

			current_yaw = atan2(ego_last_y - ego_last_last_y, ego_last_x - ego_last_last_x);

			point_x.push_back(ego_last_last_x);
		  	point_x.push_back(ego_last_x);
			
		  	point_y.push_back(ego_last_last_y);
		 	point_y.push_back(ego_last_y);

		  }


		  lane = ego_lane + target_lane;

		  vector<double> d_projection = get_lane_change_d(ego_lane,target_lane); 
		  
		  std::cout << "  ego_lane  " << car_params.car_d << "     "  << "   target_lane   " << target_lane << "   ";
		  vector<double> next_wp0 = getXY(car_params.car_s + 45, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  //vector<double> next_wp1 = getXY(car_params.car_s + 25, d_projection[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_params.car_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  //vector<double> next_wp3 = getXY(car_params.car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  //vector<double> next_wp4 = getXY(car_params.car_s + 55, d_projection[4], map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  //vector<double> next_wp5 = getXY(car_params.car_s + 70, d_projection[5], map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp6 = getXY(car_params.car_s + 135, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  //vector<double> next_wp7 = getXY(car_params.car_s + 85, d_projection[7], map_waypoints_s, map_waypoints_x, map_waypoints_y);

		  point_x.push_back(next_wp0[0]);
          //point_x.push_back(next_wp1[0]);
          point_x.push_back(next_wp2[0]);
		  //point_x.push_back(next_wp3[0]);
		  //point_x.push_back(next_wp4[0]);
		  //point_x.push_back(next_wp5[0]);
		  point_x.push_back(next_wp6[0]);
		  //point_x.push_back(next_wp7[0]);
		  
          point_y.push_back(next_wp0[1]);
          //point_y.push_back(next_wp1[1]);
          point_y.push_back(next_wp2[1]);
		  //point_y.push_back(next_wp3[1]);
		  //point_y.push_back(next_wp4[1]);
		  //point_y.push_back(next_wp5[1]);
		  point_y.push_back(next_wp6[1]);
		  //point_y.push_back(next_wp7[1]);

          //std::cout << "car_params.car_s " << car_params.car_s << std::endl;
		  for ( int i = 0; i < point_x.size(); i++ ) {
			double shift_x = point_x[i] - ego_last_x;
			double shift_y = point_y[i] - ego_last_y;
//		    std::cout << "shift " << shift_x ;
		    //std::cout << " old x = " << point_x[i] << " y= " << point_y[i];
			point_x[i] = shift_x * cos(0 - current_yaw) - shift_y * sin(0 - current_yaw);
			point_y[i] = shift_x * sin(0 - current_yaw) + shift_y * cos(0 - current_yaw);
		    //std::cout << " new x = " << point_x[i] << " y= " << point_y[i] << std::endl;
		  }

		  tk::spline s;
		  s.set_points(point_x, point_y);
		  
		  double target_x = 45.0;
		  double target_y = s(target_x);
		  double target_dist = sqrt(target_x*target_x + target_y*target_y);
		  double x_add_on = 0.0;

		  //std::cout << " car_params.car_speed " << car_params.car_speed;


		  for (int i = 0; i < 50 - prev_list_size; i++) {
			current_vel = get_safe_velocity(ego_lane,is_left_lane_safe, is_current_safe, is_right_lane_safe);
			//std::cout << "	N " << N << std::endl;
			 double N = target_dist/(0.02*current_vel);
//			 std::cout << "  N " << N << std::endl;

			 double x_point = x_add_on + (target_x)/N;
	         double y_point = s(x_point);

			 x_add_on = x_point;
			 double x_ref = x_point;
			 double y_ref = y_point;

			 x_point = x_ref * cos(current_yaw) - y_ref * sin(current_yaw);
			 y_point = x_ref * sin(current_yaw) + y_ref * cos(current_yaw);

			 x_point += ego_last_x;
			 y_point += ego_last_y;
			//std::cout << " x_point x = " << x_point << " y_point= " << y_point<< std::endl;
			 next_x_vals.push_back(x_point);
			 next_y_vals.push_back(y_point);
		  }
//		  for (int i = 0; i < 6; i++) {
//			  std::cout << "car " << nearest_car[i] << " ";
//		  }
//		  std::cout << std::endl;

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

