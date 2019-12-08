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
	void greedy_solve();
private:
	std::vector<City> cities;
	std::vector<int> permutation;
	std::vector<TSP>* reduction(int a);
	std::vector<TSP>* my_reduction(int a);
	void restoration(const std::vector<TSP>* subtasks);
	std::vector<int>* external_permutation(const std::vector<TSP>* subtasks);
	double distance(const Point& p1, const Point& p2);
	double distance(const City& c1, const City& c2);
	double distance(const City* c1, const City* c2);
	Point get_center() const;
	Point get_center(const std::vector<int>& cities_idx) const;
	void insert_cycle_to_permutation(int local_idx_curr_begin, int local_idx_curr_end,
		const TSP& subtask);
	int find_local_city_idx(int number);
	int find_permutation_idx(const TSP& task, int local_idx);
};

