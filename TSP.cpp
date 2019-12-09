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

std::vector<int>* TSP::external_permutation(const std::vector<TSP>* subtasks) {
	// find the best permutation for external connections
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
	std::vector<int>* best_permutation = new std::vector<int>();
	best_permutation->reserve(size);
	double path_length = 0;
	do {
		path_length = 0;
		int current = clusters_permutation[0];
		int next;
		for (int i = 1; i < size; ++i) {
			next = clusters_permutation[i];
			path_length += distance(clusters_centers[current],
				clusters_centers[next]);
			current = next;
		}
		path_length += distance(clusters_centers[current],
			clusters_centers[clusters_permutation[0]]);
		if (min_length > path_length) {
			min_length = path_length;
			*best_permutation = clusters_permutation;
		}
	} while (std::next_permutation(clusters_permutation.begin(), clusters_permutation.end()));
	return best_permutation;
}

void TSP::restoration(const std::vector<TSP>* subtasks)
{
	//external connections
	auto best_permutation = std::unique_ptr<std::vector<int>>(external_permutation(subtasks));

	//clusters union
	int size = cities.size();
	permutation = std::vector<int>();
	permutation.reserve(size);
	int current = (*best_permutation)[0];
	int next = (*best_permutation)[1];
	auto* subtask_curr = &(*subtasks)[current];
	auto* subtask_next = &(*subtasks)[next];
	int size_curr = subtask_curr->cities.size();
	int size_next = subtask_next->cities.size();
	double min_dist = DBL_MAX;
	int local_idx_next_begin = -1;
	int local_idx_curr_begin = -1;
	for (int i_curr = 0; i_curr < size_curr; ++i_curr)
		for (int i_next = 0; i_next < size_next; ++i_next) {
			double d = distance(subtask_curr->cities[i_curr],
				subtask_next->cities[i_next]);
			if (min_dist > d) {
				min_dist = d;
				local_idx_curr_begin = i_curr;
				local_idx_next_begin = i_next;
			}
		}
	permutation.push_back(find_local_city_idx(subtask_curr->cities[local_idx_curr_begin].number));
	permutation.push_back(find_local_city_idx(subtask_next->cities[local_idx_next_begin].number));

	//local indexes in the first cluster (*subtasks)[best_permutation[0]],
	//one of them will be the last in the Hamilton cycle
	int last_pair[2];
	int permutation_idx = find_permutation_idx(*subtask_curr, local_idx_curr_begin);
	last_pair[0] = permutation_idx - 1 < 0 ?
		subtask_curr->permutation[size_curr - 1] :
		subtask_curr->permutation[permutation_idx - 1];
	last_pair[1] = permutation_idx + 1 >= size_curr ?
		subtask_curr->permutation[0] :
		subtask_curr->permutation[permutation_idx + 1];
	//the local index of the city, which is start and end of the Hamilton cycle
	int local_idx_end = local_idx_curr_begin;

	current = next;
	for (int i = 2; i < (*best_permutation).size(); ++i) {
		local_idx_curr_begin = local_idx_next_begin;
		subtask_curr = &(*subtasks)[current];
		size_curr = subtask_curr->cities.size();
		int local_idx_curr_end = -1;
		int current_pair[2];
		permutation_idx = find_permutation_idx(*subtask_curr, local_idx_curr_begin);
		current_pair[0] = permutation_idx - 1 < 0 ?
			subtask_curr->permutation[size_curr - 1] :
			subtask_curr->permutation[permutation_idx - 1];
		current_pair[1] = permutation_idx + 1 >= size_curr ?
			subtask_curr->permutation[0] :
			subtask_curr->permutation[permutation_idx + 1];
		double min_dist = DBL_MAX;
		next = (*best_permutation)[i];
		subtask_next = &(*subtasks)[next];
		size_next = subtask_next->cities.size();
		for (int i_curr : current_pair)
			for (int i_next = 0; i_next < size_next; ++i_next) {
				double d = distance(subtask_curr->cities[i_curr], subtask_next->cities[i_next]);
				if (min_dist > d) {
					min_dist = d;
					local_idx_curr_end = i_curr;
					local_idx_next_begin = i_next;
				}
			}
		insert_cycle_to_permutation(local_idx_curr_begin, local_idx_curr_end,
			(*subtasks)[current]);
		permutation.push_back(find_local_city_idx(subtask_next->cities[local_idx_next_begin].number));
		current = next;
	}

	//end of restoration
	subtask_curr = &(*subtasks)[current];
	size_curr = subtask_curr->cities.size();
	local_idx_curr_begin = local_idx_next_begin;
	int local_idx_curr_end = -1;
	int current_pair[2];
	permutation_idx = find_permutation_idx(*subtask_curr, local_idx_curr_begin);
	current_pair[0] = permutation_idx - 1 < 0 ?
		subtask_curr->permutation[size_curr - 1] :
		subtask_curr->permutation[permutation_idx - 1];
	current_pair[1] = permutation_idx + 1 >= size_curr ?
		subtask_curr->permutation[0] :
		subtask_curr->permutation[permutation_idx + 1];
	min_dist = DBL_MAX;
	next = (*best_permutation)[0];
	subtask_next = &(*subtasks)[next];
	for (int i_curr : current_pair)
		for (int i_next : last_pair) {
			double d = distance(subtask_curr->cities[i_curr], subtask_next->cities[i_next]);
			if (min_dist > d) {
				min_dist = d;
				local_idx_curr_end = i_curr;
				local_idx_next_begin = i_next;
			}
		}
	insert_cycle_to_permutation(local_idx_curr_begin, local_idx_curr_end,
		(*subtasks)[current]);
	permutation.push_back(find_local_city_idx(subtask_next->cities[local_idx_next_begin].number));

	//last cycle
	local_idx_curr_begin = local_idx_next_begin;
	local_idx_curr_end = local_idx_end;
	insert_cycle_to_permutation(local_idx_curr_begin, local_idx_curr_end,
		(*subtasks)[next]);
}

void TSP::solve(int a, int b)
{
	//reduction
	std::unique_ptr<std::vector<TSP>> subtasks;
	if (b > 0 && cities.size() > a) {
		subtasks = std::unique_ptr<std::vector<TSP>>(my_reduction(a));
		for (auto& sub : *subtasks)
			sub.solve(a, b - 1);
	}
	else
	{
		greedy_solve();
		return;
	}
	restoration(subtasks.get());
}

std::vector<int> TSP::get_solution()
{
	return permutation;
}

double TSP::get_total_length()
{
	double dist = 0;
	int current = permutation[0];
	for (int i = 1; i < permutation.size(); ++i) {
		int next = permutation[i];
		dist += distance(cities[current], cities[next]);
		current = next;
	}
	dist += distance(cities[current], cities[permutation[0]]);
	return dist;
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
			for (int center : centers)
				if (i != center)
					d += distance(cities[i], cities[center]);
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
		int nearest_center_idx = -1;
		for (int j = 0; j < centers.size(); ++j) {
			if (i == centers[j]) {
				min_dist = 0;
				nearest_center_idx = j;
				break;
			}
			double d = distance(cities[i], cities[centers[j]]);
			if (min_dist > d) {
				min_dist = d;
				nearest_center_idx = j;
			}
		}
		clusters[nearest_center_idx].push_back(i);
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

std::vector<TSP>* TSP::my_reduction(int a)
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
			for (int center : centers)
				if (i != center)
					d += distance(cities[i], cities[center]);
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

	struct Cluster {
		Point center;
		std::vector<int> cities_idx;
		Cluster() {}
		void add_city_idx(int city_idx) {
			cities_idx.push_back(city_idx);
		}
	};
	auto already_used = [&centers](int idx) {
		for (auto center : centers)
			if (center == idx)
				return true;
		return false;
	};

	std::vector<Cluster> clusters(a);
	for (int i = 0; i < a; ++i) {
		clusters[i].add_city_idx(centers[i]);
		clusters[i].center = Point(cities[centers[i]].x, cities[centers[i]].y);
	}
	for (int i = 0; i < cities.size(); ++i) {
		if (already_used(i))
			continue;
		double min_dist = DBL_MAX;
		int nearest_cluster_idx = -1;
		Point curr_city = Point(cities[i].x, cities[i].y);
		for (int j = 0; j < clusters.size(); ++j) {			
			double d = distance(curr_city, clusters[j].center);
			if (min_dist > d) {
				min_dist = d;
				nearest_cluster_idx = j;
			}
		}
		clusters[nearest_cluster_idx].add_city_idx(i);
		clusters[nearest_cluster_idx].center = 
			get_center(clusters[nearest_cluster_idx].cities_idx);
	}

	std::vector<TSP>* subtasks = new std::vector<TSP>();
	subtasks->reserve(clusters.size());
	for (auto cluster : clusters) {
		subtasks->emplace_back(TSP());
		for (int i = 0; i < cluster.cities_idx.size(); ++i)
			subtasks->back().add_city(cities[cluster.cities_idx[i]]);
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

double TSP::distance(const City* c1, const City* c2)
{
	return distance(Point(c1->x, c1->y), Point(c2->x, c2->y));
}

TSP::Point TSP::get_center() const
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

TSP::Point TSP::get_center(const std::vector<int>& cities_idx) const
{
	std::vector<Point> cluster;
	cluster.reserve(cities_idx.size());
	for (auto city_idx : cities_idx)
		cluster.emplace_back(Point(cities[city_idx].x, cities[city_idx].y));

	double accum_x = std::accumulate(cluster.begin(), cluster.end(), 0.0,
		[](double sum, const auto& point) {
			return sum + point.x;
		});
	double accum_y = std::accumulate(cluster.begin(), cluster.end(), 0.0,
		[](double sum, const auto& point) {
			return sum + point.y;
		});
	return Point(accum_x / cluster.size(), accum_y / cluster.size());
}

void TSP::insert_cycle_to_permutation(int local_idx_curr_begin, int local_idx_curr_end,
	const TSP& subtask)
{
	if (subtask.cities.size() == 1)
		return;
	int size_curr = subtask.cities.size();
	int permutation_idx = find_permutation_idx(subtask, local_idx_curr_begin);	
	int step = 0;
	int local_idx_next = permutation_idx + 1 >= size_curr ?
		subtask.permutation[0] : subtask.permutation[permutation_idx + 1];
	if (local_idx_next == local_idx_curr_end)
		step = -1;
	else
		step = 1;
	local_idx_next = subtask.permutation[permutation_idx];
	do {
		permutation_idx += step;
		if (permutation_idx == size_curr)
			permutation_idx = 0;
		if (permutation_idx == -1)
			permutation_idx = size_curr - 1;
		local_idx_next = subtask.permutation[permutation_idx];
		permutation.push_back(find_local_city_idx(subtask.cities[local_idx_next].number));
	} while (local_idx_next != local_idx_curr_end);
}

int TSP::find_local_city_idx(int number)
{
	for (int i = 0; i < cities.size(); ++i) 
		if (number == cities[i].number)
			return i;
}

int TSP::find_permutation_idx(const TSP& task, int local_idx) {
	for (int i = 0; i < task.cities.size(); ++i)
		if (task.permutation[i] == local_idx)
			return i;
}