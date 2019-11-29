#pragma once
#include <vector>
#include <memory>

class TSP
{
public:
	struct City {
		int number;
		double x;
		double y;
		City(int n, double x, double y) :
			number(n), x(x), y(y) {}
	};
	struct Point {
		double x;
		double y;
		Point(double x = 0, double y = 0) :
			x(x), y(y) {}
	};
	TSP() {};
	TSP(const std::vector<City>& cities);
	void add_city(City city);
	void solve(int a, int b);
	const int* get_solution();
	std::vector<TSP>* reduction(int a);
	void greedy_solve();
	void restoration();
	double distance(const Point& p1, const Point& p2);
	double distance(const City& c1, const City& c2);
	Point get_center();
private:
	std::vector<City> cities;
	std::vector<int> permutation;
};

