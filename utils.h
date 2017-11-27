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

namespace carnd
{
	using namespace std;

	// For converting back and forth between radians and degrees.
	constexpr double pi() { return M_PI; }
	double deg2rad(double x) { return x * pi() / 180; }
	double rad2deg(double x) { return x * 180 / pi(); }

	// Converting between miles/hour to meters/second
	constexpr double MPH2MPS = 0.44704;
	double mph2mps(double x) { return x * MPH2MPS; }
	double mps2mph(double x) { return x / MPH2MPS; }

	// Infinity is useful
  	constexpr double INF = numeric_limits<double>::infinity();

  	// Converting between miles to meter
    static constexpr double MILE2METER = 1609.34;
    constexpr double miles2meters(double x) { return x * MILE2METER; }
    constexpr double meters2miles(double x) { return x / MILE2METER; }

	// dot product
	double dot(double x1, double y1, double x2, double y2) 
	{
	return x1 * x2 + y1 * y2;
	}

	// 2d vector norm
	double norm(double x, double y)
	{
		return sqrt(x * x + y * y);
	} 
	
	// euclidean distance
	double distance(double x1, double y1, double x2, double y2)
	{
		return norm(x2 - x1, y2 - y1);
	}

	// interpolated curve
    struct spline_curve {
    	void fit(vector<double> s, vector<double> x, vector<double> y);
    	inline double x(double s) const { return s_x_(s); }
    	inline double y(double s) const { return s_y_(s); }
    private:
    	tk::spline s_x_;
    	tk::spline s_y_;
    };

    void spline_curve::fit(vector<double> s, vector<double> x, vector<double> y) {
    	// TODO: Check loops where s is not monotonic
    	s_x_.set_points(s, x);
    	s_y_.set_points(s, y);
    }
	
}