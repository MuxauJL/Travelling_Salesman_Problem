#include <iostream>
#include "TSP.h"
#include <fstream>

int main() {
	/*std::vector<TSP::City> cities;
	cities.emplace_back(0, 0, 0);
	cities.emplace_back(1, 0, 1);
	cities.emplace_back(2, 1, 0);
	cities.emplace_back(3, 1, 1);
	TSP tsp(cities);
	auto ctr = tsp.get_center();
	printf("(%f, %f)", ctr.x, ctr.y);*/
	//const int* p = tsp.get_solution();
	std::string dir = "Task5\\";
	std::vector<std::string> files = {
		"task_4_1_n38.txt",
		"task_4_2_n131.txt",
		"task_4_3_n237.txt",
		"task_4_4_n662.txt",
		"task_4_5_n734.txt",
		"task_4_6_n8246.txt"
	};

	for (size_t i = 0; i < files.size(); ++i) {
		std::ifstream in(dir + files[i]);
		int size;
		TSP tsp;
		if (in.is_open()) {
			in >> size;
			int number;
			double x, y;
			for (int j = 0; j < size; ++j) {
				in >> number;
				in >> x >> y;
				tsp.add_city(TSP::City(number, x, y));
			}
		}
		int a = 10;
		int b = 3;
		tsp.solve(a, b);
		printf("%d) distance: %f\n", i + 1, tsp.get_total_length());
		in.close();
	}
	return 0;
}