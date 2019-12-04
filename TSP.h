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
		City() = default;
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
	std::vector<int> get_solution();
	double get_total_length();
	std::vector<TSP>* reduction(int a);
	void greedy_solve();
	void restoration(const std::vector<TSP> * subtasks);
	double distance(const Point& p1, const Point& p2);
	double distance(const City& c1, const City& c2);
	double distance(const City* c1, const City* c2);
	Point get_center() const;
private:
	std::vector<City> cities;
	std::vector<int> permutation;
	void insert_cycle_to_permutation(int local_idx_curr_begin, int local_idx_curr_end,
		const TSP& subtask);
	int find_local_city_idx(int number);
	int find_permutation_idx(const TSP& task, int local_idx);
};

