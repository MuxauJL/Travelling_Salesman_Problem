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
		path_length += distance(clusters_centers.back(), clusters_centers.front());
		if (min_length > path_length) {
			min_length = path_length;
			best_permutation = clusters_permutation;
		}
	} while (std::next_permutation(clusters_permutation.begin(), clusters_permutation.end()));

	//clusters union
	size = cities.size();
	permutation = std::vector<int>(size);
	permutation.reserve(size);
	int current = best_permutation[0];
	int next = best_permutation[1];
	auto& cities_curr = (*subtasks)[current].cities;
	auto& cities_next = (*subtasks)[next].cities;
	int size_curr = cities_curr.size();
	int size_next = cities_next.size();
	double min_dist = DBL_MAX;
	//std::pair<int, int> nearest;
	int local_idx_next_begin = -1;
	int local_idx_curr_begin = -1;
	for (int i_curr = 0; i_curr < size_curr; ++i_curr)
		for (int i_next = 0; i_next < size_next; ++i_next) {
			double d = distance(cities_curr[i_curr],
				cities_next[i_next]);
			if (min_dist > d) {
				min_dist = d;
				/*nearest.first = cities_curr[i_curr].number;
				nearest.second = cities_next[i_next].number;*/
				local_idx_curr_begin = i_curr;
				local_idx_next_begin = i_next;
			}
		}
	permutation.push_back(cities_curr[local_idx_curr_begin].number);
	permutation.push_back(cities_next[local_idx_next_begin].number);

	//local indexes in the first cluster (*subtasks)[best_permutation[0]],
	//one of them will be the last in the Hamilton cycle
	int last_pair[2];
	last_pair[0] = local_idx_curr_begin - 1 < 0 ? cities_curr.size() - 1 : local_idx_curr_begin - 1;
	last_pair[1] = local_idx_curr_begin + 1 >= cities_curr.size() ? 0 : local_idx_curr_begin + 1;
	//the local index of the city, which is start and end of the Hamilton cycle
	int local_idx_end = local_idx_curr_begin;

	current = next;
	local_idx_curr_begin = local_idx_next_begin;
	for (int i = 2; i < best_permutation.size(); ++i) {
		cities_curr = (*subtasks)[current].cities;
		int local_idx_curr_end = -1;
		int current_pair[2];
		current_pair[0] = local_idx_curr_begin - 1 < 0 ? cities_curr.size() - 1 : local_idx_curr_begin - 1;
		current_pair[1] = local_idx_curr_begin + 1 >= cities_curr.size() ? 0 : local_idx_curr_begin + 1;
		double min_dist = DBL_MAX;
		next = best_permutation[i];
		cities_next = (*subtasks)[next].cities;
		size_next = cities_next.size();
		for (int i_curr : current_pair)
			for (int i_next = 0; i_next < size_next; ++i_next) {
				double d = distance(cities_curr[i_curr], cities_next[i_next]);
				if (min_dist > d) {
					min_dist = d;
					/*nearest.first = cities_curr[i_curr].number;
					nearest.second = cities_next[i_next].number;*/
					local_idx_curr_end = i_curr;
					local_idx_next_begin = i_next;
				}
			}
		insert_cycle_to_permutation(local_idx_curr_begin, local_idx_curr_end,
			cities_curr);
		permutation.push_back(cities_next[local_idx_next_begin].number);
		current = next;
	}

	//end of restoration
	cities_curr = (*subtasks)[current].cities;
	local_idx_curr_begin = local_idx_next_begin;
	int local_idx_curr_end = -1;
	int current_pair[2];
	current_pair[0] = local_idx_curr_begin - 1 < 0 ? cities_curr.size() - 1 : local_idx_curr_begin - 1;
	current_pair[1] = local_idx_curr_begin + 1 >= cities_curr.size() ? 0 : local_idx_curr_begin + 1;
	min_dist = DBL_MAX;
	next = best_permutation[0];
	cities_next = (*subtasks)[next].cities;
	for (int i_curr : current_pair)
		for (int i_next : last_pair) {
			double d = distance(cities_curr[i_curr], cities_next[i_next]);
			if (min_dist > d) {
				min_dist = d;
				/*nearest.first = cities_curr[i_curr].number;
				nearest.second = cities_next[i_next].number;*/
				local_idx_curr_end = i_curr;
				local_idx_next_begin = i_next;
			}
		}
	insert_cycle_to_permutation(local_idx_curr_begin, local_idx_curr_end,
		cities_curr);
	permutation.push_back(cities_next[local_idx_next_begin].number);

	//last cycle
	cities_curr = (*subtasks)[next].cities;
	local_idx_curr_begin = local_idx_next_begin;
	local_idx_curr_end = local_idx_end;
	insert_cycle_to_permutation(local_idx_curr_begin, local_idx_curr_end,
		cities_curr);
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

void TSP::insert_cycle_to_permutation(int local_idx_curr_begin, int local_idx_curr_end,
	const std::vector<TSP::City>& cities_curr)
{
	int step = 0;
	int local_idx_next = local_idx_curr_begin + 1 >= cities_curr.size() ? 0 : local_idx_curr_begin + 1;
	if (local_idx_next == local_idx_curr_end)
		step = -1;
	else
		step = 1;
	local_idx_next = local_idx_curr_begin;
	int size_curr = cities_curr.size();
	do {
		local_idx_next += step;
		if (local_idx_next == size_curr)
			local_idx_next = 0;
		if (local_idx_next == -1)
			local_idx_next = size_curr - 1;
		permutation.push_back(cities_curr[local_idx_next].number);
	} while (local_idx_next != local_idx_curr_end);
}
