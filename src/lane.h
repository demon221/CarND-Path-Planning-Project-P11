#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <random>


namespace carnd
{
	using namespace std;

	struct Lane
	{
		// There are three lanes numbered in 0, 1, 2 from left to right
		int lane_count = 3;
		// Lane width
		double lane_width = 4;
		// Lane speed limitation
		double speed_limit_mph = 50;

		double road_width = lane_width * lane_count;

		// Lane center from middle line
		double lane_center(int lane) const;

		// Safe zone with a bias in the left and right lanes
		double safe_lane_center(int lane) const;

		// Lane at a given d from the road center
		int lane_at(double d) const;

		// Distance to a given lane
		double distance_to_lane(double d, int lane) const;
	
	};

	// Lane center from middle line
	double Lane::lane_center(int lane) const
	{
		return (0.5 + lane) * lane_width;
	}

	// Safe zone with a bias in the left and right lanes
	double Lane::safe_lane_center(int lane) const
	{
		double center = lane_center(lane);
		if (lane == 0) // left lane
			center += 0;
		else if (lane == lane_count - 1) // right lane
			center -= 0;
		return center;
	}

	// Lane at a given d from the road center
	int Lane::lane_at(double d) const
	{
		if (d < 0){
			cout << "ERROR: car beyond the left lane !!!" << endl;
			return -1; 
		}
		else if (d > road_width){
			cout << "ERROR: car beyond the right lane !!!" << endl;
			return -2;
		}
		return floor(d / lane_width);
	}

	// Distance to a given lane
	double Lane::distance_to_lane(double d, int lane) const
	{
		return lane_center(lane) - d;
	}

} // namespace carnd