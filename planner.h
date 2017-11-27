#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <random>
#include "spline.h"
#include "roadmap.h"
#include "lane.h"
#include "utils.h"


namespace carnd
{
	using namespace std;

	// Car objects from sensor fusion data
	struct car_t
	{
		int id;
		double x, y, vx, vy, s, d;
	};

	// Path trajectory points
	struct path_t
	{
		vector<double> x, y;

		size_t size() const { return x.size(); }

		void append(const xy_t xy){
			x.push_back(xy.x);
			y.push_back(xy.y);
		}

		void append(const double x_, const double y_){
			x.push_back(x_);
			y.push_back(y_);
		}
	};

	// Telemetry data with ego motion and its path
	// Car objects from sensor fusion data
	struct ego_t
	{
		double x, y, s, d, yaw, v; // ego motion
		path_t previous_path;
		sd_t end_path;
		vector<car_t> cars; // cars from sensor fusion
	};

	struct lane_info_t
	{
		int front_car = -1;
		int back_car = -1;
		double front_gap = 1000;
		double front_speed = 1000;
		double front_gap_next = 1000;
		double back_gap = -1000;
		double back_speed = -1000;
		double back_gap_next = -1000;
		bool feasible = true;

		bool is_clear() const { return feasible && front_car < 0; }
	};

	// Path planner
	struct PathPlanner
	{
		RoadMap roadmap;
		Lane lane;

		vector<lane_info_t> lane_info;

		double accel = 0.1; // m/s^2
		double emergy_accel = 0.2; // m/s^2
		bool warning_collision = false;

		// Number of points
		int n_path_points = 50;

		// Reference point for new path
		double ref_x, ref_y, ref_x_prev, ref_y_prev;
		double ref_s, ref_d;
		double ref_yaw, ref_v;
		int ref_lane;
		int ref_points;

		// Lap tracking for the ego
		size_t ego_laps = 0;
		size_t ego_laps_tick = 0;
		bool ego_passed_zero_s = false;
		sd_t ego_start_position;

		// Lane change parameter
		double lane_horizon = 50; //m
		double lane_change_front_buffer = 10; //m
		double lane_change_back_buffer = -10; //m, backward minus value
		double lane_emergy_front_buffer = 5; //m

		// Target lane for next path
		int changing_lane = -1;
		int target_lane = 1;
		double target_speed = 0;
		

		enum class STATE { START = 0, KEEPLANE = 1, PRELANECHANGE = 2, LANECHANGE = 3 };
		STATE state_ = STATE::START;
		double state_s_;

	public:
		void initialize(const string & map_file_);

		void reset();

		// Run the planner with telemetry data to generate next trajectory
		// dt is the simulator period
		void run(const ego_t & ego, path_t & path, double dt);

	protected:
		void set_state(const ego_t & ego, STATE new_state);
		int get_best_lane() const;

		void get_reference(const ego_t & ego, double dt);
		void track_lap(const ego_t & ego);
		void process_sensor_fusion(const ego_t & ego, double dt);
		void create_plan(const ego_t & ego, double dt);
		void collision_avoidance();
		void speed_control();
		void create_trajectory(const ego_t & ego, 
						  const int target_lane, 
						  const double target_speed, 
						  path_t & path, 
						  double dt);

	};

	void PathPlanner::initialize(const string & map_file_)
	{
		roadmap.load(map_file_);
	}

	void PathPlanner::reset()
	{
		warning_collision = false;
		target_lane = 1;
		target_speed = 0;
		ref_points = 0;
		ego_laps = 0;
		ego_laps_tick = 0;
		ego_passed_zero_s = false;
		state_ = STATE::START;
		state_s_ = 0;
	}

	void PathPlanner::set_state(const ego_t & ego, STATE new_state)
	{
		if (state_ != new_state)
		{
			state_ = new_state;
			state_s_ = ref_s;
		}
	}

	int PathPlanner::get_best_lane() const
	{
		if (lane_info[target_lane].is_clear())
			return target_lane;

		vector<int> lanes(lane.lane_count);
		iota(lanes.begin(), lanes.end(), 0);

		// Search for the best lane
		sort(lanes.begin(), lanes.end(), [&](int i, int j)
		{
			auto & lane_i = lane_info[i];
			auto & lane_j = lane_info[j];

			auto i_closest_to_ref_lane = abs(i - ref_lane) <= abs(j - ref_lane);

			// Clear lane result
			if (lane_i.is_clear())
			{
				if (lane_j.is_clear())
					return i_closest_to_ref_lane;
				else
					return true;
			}
			else if (lane_j.is_clear())
				return false;

			// Calculate lane's velocity allowance: INF, front speed or back speed
			const double i_v = lane_i.front_gap >= lane_horizon ?
							   INF :
							   lane_i.back_gap > lane_change_back_buffer ?
							   lane_i.back_speed: lane_i.front_speed;
			const double j_v = lane_j.front_gap >= lane_horizon ?
							   INF :
							   lane_j.back_gap > lane_change_back_buffer ?
							   lane_j.back_speed: lane_j.front_speed;

			// When lane's velocity is the same, then compare the front gap
			if ((!isfinite(i_v) && !isfinite(j_v)) || fabs(i_v - j_v) < 0.5)
				return lane_i.front_gap >= lane_j.front_gap;
			// Compare the lane's velocity
			else
				return i_v > j_v;
		});

		return lanes[0];

	}


	// Planner main function, run the planner with telemetry data to generate next trajectory
	void PathPlanner::run(const ego_t & ego, path_t & path, double dt)
	{
		// 1a. Get reference point of ego motion
		get_reference(ego, dt);

		// 1b. Track laps to check if it's a new lap
		track_lap(ego);

		// 2. Environment analysis, process the data from sensor fusion with prediction
		process_sensor_fusion(ego, dt);

		// 3. Behavior plan, create plann for target lane and speed
		create_plan(ego, dt);

		// 4. Collision avoid
		collision_avoidance();

		// 5. Speed control
		speed_control();

		// 6. Generate final trajectory
		create_trajectory(ego, target_lane, target_speed, path, dt);

	}

	// 1a. Get reference point of ego motion
	void PathPlanner::get_reference(const ego_t & ego, double dt)
	{
		const int planned_size = ego.previous_path.size();

		// If previous path is empty, then use current ego position
		if (planned_size < 2)
		{
			ref_x = ego.x;
			ref_y = ego.y;
			ref_s = ego.s;
			ref_d = ego.d;
			ref_yaw = ego.yaw;
			ref_v = ego.v;

			ref_x_prev = ref_x - cos(ref_yaw);
			ref_y_prev = ref_y - sin(ref_yaw);
		}
		else // use previous path
		{
			ref_x = *(ego.previous_path.x.end() - 1);
			ref_y = *(ego.previous_path.y.end() - 1);

			ref_x_prev = *(ego.previous_path.x.end() - 2);
			ref_y_prev = *(ego.previous_path.y.end() - 2);

			ref_s = ego.end_path.s;
			ref_d = ego.end_path.d;

			ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
			ref_v = distance(ref_x_prev, ref_y_prev, ref_x, ref_y) / dt;

		}

		// Get reference lane
		ref_lane = lane.lane_at(ref_d);
		// Keep track the size of the previous path
		ref_points += n_path_points - planned_size;

	} // end PathPlanner::get_reference()

	
	// 1b. Track laps to check if it's a new lap
	void PathPlanner::track_lap(const ego_t & ego)
	{
		// Check if new lap
		if (ego_laps_tick == 0)
		{
			ego_start_position = {ego.s, ego.d};
		}
		// Check if passed zero point of road
		if (ego.s < ego_start_position.s)
		{
			ego_passed_zero_s = true;
		}
		// Add laps and reset tick when new lap
		if (ego_passed_zero_s && ego.s > ego_start_position.s)
		{
			ego_laps ++;
			ego_laps_tick = 0;
			ego_passed_zero_s = false;

			cout << "############### New Lap! ###################" << endl;
		}
		else
		{
			cout << "____________________________________________" << endl;

		}

		ego_laps_tick ++;

		cout << endl
			 << " LAP = " << (ego_laps + 1)
		     << " LANE = " << lane.lane_at(ego.d)
		     << " (s= " << fixed << setprecision(1) << ego.s
		     << ", d= " << fixed << setprecision(1) << ego.d << ")"
		     << " PLANNED " << ego.previous_path.size() << " points."
		     << endl;
	} // end PathPlanner::track_lap()

	
	// 2. Environment analysis, process the data from sensor fusion with prediction
	void PathPlanner::process_sensor_fusion(const ego_t & ego, double dt)
	{
		cout << "##Sensor Fusion##" << endl;
		lane_info.clear();
		lane_info.resize(lane.lane_count);

		const int planned_size = ego.previous_path.size();

		// Analysis each car objects in sensor fusion
		for(auto & car : ego.cars)
		{
			// Get cars' lane
			int car_lane = lane.lane_at(car.d);
			// Only cars in same direction
			if (car_lane >= 0)
			{
				lane_info_t & laneinfo = lane_info[car_lane];

				// Predict car position assuming constant speed
				double car_speed = norm(car.vx, car.vy);
				double car_next_s = car.s + car_speed * planned_size * dt;
				// Check if it's in front or back
				bool in_front = car.s > ref_s;
				// Absolute s distance from ego to car
				double car_gap = car.s - ref_s;
				double car_gap_next = car_next_s - ref_s;

				cout << " CAR " << setw(2) << car.id
					 << " lane=" << car_lane
					 << " v=" << setw(4) << mps2mph(car_speed)
					 << " gap=" << setw(4) << car_gap
					 << " gap'=" << setw(4) << car_gap_next
					 << endl;

				// Check if distance is under buffer
				// Check front
				if (in_front == true)
				{
					if (car_gap < laneinfo.front_gap)
					{
						laneinfo.front_car = car.id;
						laneinfo.front_gap = car_gap; 
						laneinfo.front_speed = car_speed;
						laneinfo.front_gap_next = car_gap_next;
					}
				}
				// Check back
				else if (car_gap > fmax(laneinfo.back_gap, -lane_horizon))
				{
					laneinfo.back_car = car.id;
					laneinfo.back_gap = car_gap;
					laneinfo.back_speed = car_speed;
					laneinfo.back_gap_next = car_gap_next;
				}

				// Evaluate lane feasibility
				laneinfo.feasible = (laneinfo.front_gap > lane_change_front_buffer)
								&& (laneinfo.front_gap_next > lane_change_front_buffer)
								&& (laneinfo.back_gap < lane_change_back_buffer)
								&& (laneinfo.back_gap_next < lane_change_back_buffer);

				// Store the lane info
				lane_info[car_lane] = laneinfo;

			} // end if (car_lane >= 0)
		} // end for(car_t & car : ego.cars)

		for(int i = 0; i < lane_info.size(); i++)
		{
			cout << " LANE " << setw(2) << i
			     << " front car=" << lane_info[i].front_car 
			     << " back car=" << lane_info[i].back_car
			     << " feasible=" << lane_info[i].feasible
			     << endl;
		}
	} // end PathPlanner::process_sensor_fusion()

	
	// 3. Behavior planning, create plann for target lane and speed
	void PathPlanner::create_plan(const ego_t & ego, double dt)
	{
		cout << "##Planning##" << endl;

		// Get a safety margin
		const double road_speed_limit = mph2mps(lane.speed_limit_mph) - 0.2;
		double cte = (ref_d - lane.lane_center(target_lane));

		if (state_ == STATE::START)
		{
			changing_lane = -1;
			target_lane = ref_lane;
			state_ = STATE::KEEPLANE;
			state_s_ = ego_start_position.s;
		}

		int best_lane = get_best_lane();

		cout << " ** BEST  LANE = " << best_lane << endl;
		cout << " ** REF   LANE = " << ref_lane << endl;
		cout << " ** TARGETLANE = " << target_lane << endl;

		while(true)
		{
			// s gap
			double meters_in_state = ref_s - state_s_;
			while(meters_in_state < 0)
				meters_in_state += roadmap.max_s;

			// Keep Lane
			if (state_ == STATE::KEEPLANE)
			{
				// assert( target_lane == ref_lane);
				cout << " ** KEEP LANE = " << target_lane
				     << " FOR " << setprecision(2) << meters_in_state << "m"
				     << " (cte= " << setprecision(1) << setw(4) << cte << " m)"
				     << endl;

				target_speed = road_speed_limit;

				// Evaluate lane change
				changing_lane = -1;
				// Check if current lane has slower cars in front
				if (lane_info[target_lane].front_gap < lane_change_front_buffer 
					&& lane_info[target_lane].front_speed < ego.v
					&& meters_in_state > 100)
				{
					// Change if not in the best lane
					if (lane_info[best_lane].front_speed > lane_info[target_speed].front_speed)
					{
						changing_lane = best_lane;
						set_state(ego, STATE::PRELANECHANGE);
					}
				}

				break;
			}

			// Prepare lane change
			else if (state_ == STATE::PRELANECHANGE)
			{
				// assert( changing_lane != ref_lane );

				cout << " ** PREPARE CHANGE TO LANE = " << changing_lane
				     << " FOR " << setprecision(2) << meters_in_state << "m"
				     << " (cte= " << setprecision(1) << setw(4) << cte << " m)"
				     << endl;

				target_lane = ref_lane + ((changing_lane > ref_lane) ? 1 : -1);

				// If target lane is feasible, then change lane
				if (lane_info[target_lane].feasible && meters_in_state > 5)
				{
					set_state(ego, STATE::LANECHANGE);
					
				}
				// If best lane changed and still far ahead, then cancel 
				else if (changing_lane != best_lane)
				{
					target_lane = ref_lane;
					set_state(ego, STATE::KEEPLANE);

				}
				else // If not feasible, then wait in this lane and try to slow down
				{
					if (lane_info[ref_lane].front_gap < lane_change_front_buffer)
					{
						target_lane = ref_lane;
						target_speed = fmin(target_speed, lane_info[target_lane].front_speed);
					}
				}

				break;
			}

			// Lane Change
			else if (state_ == STATE::LANECHANGE)
			{
				cout << " ** CHANGING TO LANE = " << target_lane
				     << " FOR " << setprecision(2) << meters_in_state << " m"
				     << " cte=" << setprecision(1) << setw(4) << cte << " m)"
				     << endl;

				// Accelerate when lane changning
				target_speed = fmin(target_speed, ref_v + accel);
				cte = (ref_d - lane.lane_center(target_lane));

				// Check lane change completed
				if (ref_lane == target_lane && fabs(cte) <= 0.3 && meters_in_state > 50)
				{
					// If not best lane, then prepare to lang change
					if (changing_lane >= 0 && changing_lane != ref_lane)
					{
						set_state(ego, STATE::PRELANECHANGE);
						
					}
					// Otherwise keep lane
					changing_lane = -1;
					set_state(ego, STATE::KEEPLANE);
					
				}

				// If front or back gap is so close, then abort lane change
				if (lane_info[target_lane].front_gap < lane_emergy_front_buffer)
				{
					target_lane = ref_lane;
					changing_lane = -1;
					cout << " ** ABORTING LANE CHANGE " << endl;
				}

				break;
			}

		} // end while(true)

		// Ensure target speed is inside the 0 - speed limit
		target_speed = fmax(0.0, fmin(road_speed_limit, target_speed));
	} // end PathPlanner::create_plan()

	
	// 4. Collision avoidance
	void PathPlanner::collision_avoidance()
	{
		if (lane_info[target_lane].front_gap < lane_change_front_buffer)
		{
			// Decelerate if too close
			if (lane_info[target_lane].front_gap < lane_emergy_front_buffer)
			{
				target_speed = fmax(0.0, fmin(target_speed, lane_info[target_lane].front_speed - 0.2));
				warning_collision = true;
			}
			// Follow the lead
			else
			{
				target_speed = fmax(0.0, fmin(target_speed, lane_info[target_lane].front_speed));
				warning_collision = false;
			}

			cout << " ** FOLLOW THE LEAD (" << lane_info[target_lane].front_gap << " m)" << endl;
		}
	} // end PathPlanner::collision_avoidance()

	// 5. Speed control
	void PathPlanner::speed_control()
	{
		// Decelerate
		if (target_speed < ref_v)
		{
			if (warning_collision == true)
				target_speed = fmax(target_speed, ref_v - emergy_accel);
			else
				target_speed = fmax(target_speed, ref_v - accel);
		} 
		// Accelerate
		else if (target_speed > ref_v)
			target_speed = fmin(target_speed, ref_v + accel);
		// Speed not change
		else
			target_speed = ref_v;
		// Avoid minus zero
		target_speed = fmax(target_speed, 0.0);
	}

	// 6. Generate final trajectory
	void PathPlanner::create_trajectory(const ego_t & ego, 
										const int target_lane, 
										const double target_speed, 
										path_t & path, 
										double dt)
	{
		cout << "##TRAJECTORY##" << endl
			 << " ** TARGET LANE= " << target_lane
			 << " ** TARGET SPEED= " << setprecision(1) << mps2mph(target_speed)
			 << endl;

		const double target_d = lane.safe_lane_center(target_lane);

		// Trajectory points
		path_t anchors;

		// Build a path tengent to the previous end state
		anchors.append(ref_x_prev, ref_y_prev);
		anchors.append(ref_x, ref_y);

		// Add three more points, each has 30m space
		for(int i = 1; i <=3; i++)
		{
			xy_t next_wp = roadmap.to_xy(ref_s + 30 * i, target_d);
			anchors.append(next_wp);
		}

		// Change the points to reference coordinate
		for(int i = 0; i < anchors.size(); i++)
		{
			const double dx = anchors.x[i] - ref_x;
			const double dy = anchors.y[i] - ref_y;
			anchors.x[i] = dx * cos(-ref_yaw) - dy * sin(-ref_yaw);
			anchors.y[i] = dx * sin(-ref_yaw) + dy * cos(-ref_yaw);
		}

		// Interpolate the anchors with a cubic spline
		tk::spline spline;
		spline.set_points(anchors.x, anchors.y);

		// Add previous path for continuity
		path.x.assign(ego.previous_path.x.begin(), ego.previous_path.x.end());
		path.y.assign(ego.previous_path.y.begin(), ego.previous_path.y.end());

		// Set a horizon of 30m
		const double target_x = 30;
		const double target_y = spline(target_x);
		const double target_dist = norm(target_x, target_y);

		// t = N * dt = target_dist / target_speed
		const double t = target_x / target_dist * dt;

		
		for(int i = 1; i <= n_path_points - path.x.size(); i++)
		{
			// Sample the spline curve to reach the target speed
			double x_spline = i * t * target_speed;
			double y_spline = spline(x_spline);
			// Transform back to world coordinate
			double x_ = x_spline * cos(ref_yaw) - y_spline * sin(ref_yaw) + ref_x;
			double y_ = x_spline * sin(ref_yaw) + y_spline * cos(ref_yaw) + ref_y;

			// Append the trajectory points
			path.append({x_, y_});
		}

	} // end PathPlanner::create_trajectory()

} // namespace carnd