#include <iostream>
#include "TSP.h"

int main() {
	std::vector<TSP::City> cities;
	cities.emplace_back(0, 0, 0);
	cities.emplace_back(1, 0, 1);
	cities.emplace_back(2, 1, 0);
	cities.emplace_back(3, 1, 1);
	TSP tsp(cities);
	auto ctr = tsp.get_center();
	printf("(%f, %f)", ctr.x, ctr.y);
	//const int* p = tsp.get_solution();
	return 0;
}