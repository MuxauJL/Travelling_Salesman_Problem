#include "TSP.h"
#include <algorithm>
#include <numeric>

TSP::TSP(const std::vector<City>& cities):
	cities(cities)
{
}

void TSP::add_city(City city)
{
	cities.emplace_back(city);
}

void TSP::greedy_solve()
{
	int size = cities.size();
	permutation = std::vector<int>();
	permutation.reserve(size);
	permutation.push_back(0);
	auto already_visited = [this](int idx) {
		for (auto i : permutation)
			if (idx == i)
				return true;
		return false;
	};

	while (permutation.size() < size) {
		double min_dist = DBL_MAX;
		int nearest = -1;
		int current = permutation.back();
		for (int j = 0; j < size; ++j) {
			if (already_visited(j))
				continue;
			double d = distance(cities[current], cities[j]);
			if (min_dist > d) {
				min_dist = d;
				nearest = j;
			}
		}
		permutation.push_back(nearest);
	}
}

void TSP::restoration()
{
}

void TSP::solve(int a, int b)
{
	//reduction
	std::unique_ptr<std::vector<TSP>> subtasks;
	if (b > 0 && cities.size() > 3) {
		subtasks = std::unique_ptr<std::vector<TSP>>(reduction(a));
		for (auto& sub : *subtasks)
			if (sub.cities.size() > 3)
				sub.solve(a, --b);
			else
				sub.greedy_solve();
	}
	//external connections
	int size = subtasks->size();
	std::vector<int> clusters_permutation;
	std::vector<Point> clusters_centers;
	clusters_permutation.reserve(size);
	clusters_centers.reserve(size);
	for (int i = 0; i < size; ++i) {
		clusters_permutation.push_back(i);
		clusters_centers.emplace_back((*subtasks)[i].get_center());
	}
	double min_length = DBL_MAX;
	std::vector<int> best_permutation;
	do {
		double path_length = 0;
		int current = clusters_permutation[0];
		int next;
		for (int i = 1; i < size; ++i) {
			next = clusters_permutation[i];
			path_length += distance(clusters_centers[current],
				clusters_centers[next]);
			current = next;
		}
		if (min_length > path_length) {
			min_length = path_length;
			best_permutation = clusters_permutation;
		}
	} while (std::next_permutation(clusters_permutation.begin(), clusters_permutation.end()));
	//clusters union
	//restoration();
}

const int* TSP::get_solution()
{
	return permutation.data();
}

std::vector<TSP>* TSP::reduction(int a)
{
	double max_dist = 0;
	std::pair<int, int> argmax;
	for (int i = 0; i < cities.size() - 1; ++i)
		for (int j = i + 1; j < cities.size(); ++j) {
			double d = distance(cities[i], cities[j]);
			if (d > max_dist) {
				max_dist = d;
				argmax.first = i;
				argmax.second = j;
			}
		}

	std::vector<int> centers = { argmax.first, argmax.second };
	while (centers.size() < a) {
		max_dist = 0;
		int furthest;
		for (int i = 0; i < cities.size(); ++i) {
			double d = 0;
			for (int j : centers)
				if (i != j)
					d += distance(cities[i], cities[j]);
				else {
					d = 0;
					break;
				}

			if (d > max_dist) {
				max_dist = d;
				furthest = i;
			}
		}
		centers.push_back(furthest);
	}

	std::vector<std::vector<int>> clusters(centers.size());
	for (int i = 0; i < cities.size(); ++i) {
		double min_dist = DBL_MAX;
		int nearest = -1;
		for(int j : centers){
			if (i == j) {
				min_dist = 0;
				nearest = j;
				break;
			}
			double d = distance(cities[i], cities[j]);
			if (min_dist > d) {
				min_dist = d;
				nearest = j;
			}
		}
		clusters[nearest].push_back(i);
	}

	std::vector<TSP>* subtasks = new std::vector<TSP>();
	subtasks->reserve(clusters.size());
	for (auto cluster : clusters) {
		subtasks->emplace_back(TSP());
		for (int i = 0; i < cluster.size(); ++i)
			subtasks->back().add_city(cities[cluster[i]]);
	}
	return subtasks;
}

double TSP::distance(const Point& p1, const Point& p2)
{
	double x_diff = p1.x - p2.x;
	double y_diff = p1.y - p2.y;
	return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double TSP::distance(const City& c1, const City& c2)
{
	return distance(Point(c1.x, c1.y), Point(c2.x, c2.y));
}

TSP::Point TSP::get_center()
{
	double accum_x = std::accumulate(cities.begin(), cities.end(), 0.0,
		[](double sum, const auto& city) {
			return sum + city.x;
		});
	double accum_y = std::accumulate(cities.begin(), cities.end(), 0.0,
		[](double sum, const auto& city) {
			return sum + city.y;
		});
	return Point(accum_x / cities.size(), accum_y / cities.size());
}
