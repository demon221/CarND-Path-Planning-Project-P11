#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <random>
#include "utils.h"


namespace carnd
{
	using namespace std;

	// Cartesian point
	struct xy_t { double x, y; };
	// Frenet point
	struct sd_t { double s, d; };

	// Waypoint
	struct waypoint_t 
	{
		double x, y, s, dx, dy;
	};

	// Waypoint list vector
	struct waypoints_list
	{
		vector<double> x, y, s, dx, dy;
		size_t size() const { return x.size(); }
		waypoint_t operator[](int i) const { return {x[i], y[i], s[i], dx[i], dy[i]}; }
	};

	// Roadmap structure
	struct RoadMap
	{
		// The max s value before wrapping around the track back to 0
  		double max_s = 6945.554;

  		waypoints_list waypoints;

  		// Load map waypoints from a csv file
  		void load(const string &filename);

  		// Convert cartesian to frenet
  		sd_t to_frenet(double x, double y, double theta) const;
  		// Convert frenet to cartesian
  		xy_t to_xy(double s, double d) const;

  		// Closest waypoint to x,y
  		int closet_waypoint(double x, double y) const;
  		// Next waypoint looking forward in the direction of theta
  		int next_waypoint(double x, double y, double theta) const;
	};

	// RoadMap functions

	void RoadMap::load(const string &filename)
	{
		// Waypoint map to read from file
	    ifstream in_map_(filename, ifstream::in);

	    waypoints.x.clear();
	    waypoints.y.clear();
	    waypoints.s.clear();
	    waypoints.dx.clear();
	    waypoints.dy.clear();

	    string line;
	    while (getline(in_map_, line)) {
	      istringstream iss(line);

	      double x, y, s, dx, dy;
	      iss >> x;
  		  iss >> y;
  		  iss >> s;
  		  iss >> dx;
  		  iss >> dy;

	      waypoints.x.push_back(x);
	      waypoints.y.push_back(y);
	      waypoints.s.push_back(s);
	      waypoints.dx.push_back(dx);
	      waypoints.dy.push_back(dy);
	    }

	    // Add the last point
	    waypoints.s.push_back(max_s);
	    waypoints.x.push_back(waypoints.x[0]);
	    waypoints.y.push_back(waypoints.y[0]);
	    waypoints.dx.push_back(waypoints.dx[0]);
	    waypoints.dy.push_back(waypoints.dy[0]);
	}


	// Closest waypoint to x,y of car
	int RoadMap::closet_waypoint(double x, double y) const
	{

		double closest_dist = numeric_limits<double>::max();
		int closest = 0; 

		for(int i = 0; i < waypoints.x.size(); i++) {
			double dist = distance(x, y, waypoints.x[i], waypoints.y[i]);
			if(dist < closest_dist)
			{
				closest_dist = dist;
				closest = i;
			}
		}
		return closest;
	}

	// Nearest waypoint in the direction of theta of car
	int RoadMap::next_waypoint(double x, double y, double theta) const
	{
		int next = closet_waypoint(x,y);
		xy_t next_waypoint = {waypoints.x[next], waypoints.y[next]};

		double heading = atan2(next_waypoint.y - y, next_waypoint.x - x);
		double angle = fabs(theta-heading);
		angle = min(2*M_PI - angle, angle);

		if(angle > pi()/4)
		{
		   next = (next + 1) % waypoints.x.size();
		}

		return next;
	}

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	sd_t RoadMap::to_frenet(double x, double y, double theta) const
	{
		int next = next_waypoint(x, y, theta);
		int prev = (next - 1) % waypoints.size();

		auto next_waypoint = waypoints[next];
		auto prev_waypoint = waypoints[prev];

		double n_x = next_waypoint.x - prev_waypoint.x;
		double n_y = next_waypoint.y - prev_waypoint.y;
		double x_x = x - prev_waypoint.x;
		double x_y = y - prev_waypoint.y;

		// find the projection of x onto n
		double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
		double proj_x = proj_norm*n_x;
		double proj_y = proj_norm*n_y;

		double frenet_d = distance(x_x,x_y,proj_x,proj_y);

		//see if d value is positive or negative by comparing it to a center point

		double center_x = 1000 - prev_waypoint.x;
		double center_y = 2000 - prev_waypoint.y;
		double centerToPos = distance(center_x,center_y,x_x,x_y);
		double centerToRef = distance(center_x,center_y,proj_x,proj_y);

		if(centerToPos <= centerToRef)
		{
			frenet_d *= -1;
		}

		// calculate s value
		double frenet_s = prev_waypoint.s + distance(0, 0, proj_x, proj_y);

		return {frenet_s,frenet_d};

	}

	// Transform from Frenet s,d coordinates to Cartesian x,y
	xy_t RoadMap::to_xy(double s, double d) const
	{
		int prev_wp = -1;

		while(s > waypoints[prev_wp + 1].s && (prev_wp < (int)(waypoints.size() - 1) ))
		{
			prev_wp++;
		}

		int wp2 = (prev_wp + 1) % waypoints.size();

		double heading = atan2(waypoints[wp2].y - waypoints[prev_wp].y, 
							   waypoints[wp2].x - waypoints[prev_wp].x);
		// the x,y,s along the segment
		double seg_s = (s - waypoints[prev_wp].s);

		double seg_x = waypoints[prev_wp].x + seg_s * cos(heading);
		double seg_y = waypoints[prev_wp].y + seg_s * sin(heading);

		double perp_heading = heading - M_PI/2;

		double x = seg_x + d * cos(perp_heading);
		double y = seg_y + d * sin(perp_heading);

		return {x, y};

	}

} // namespace carnd