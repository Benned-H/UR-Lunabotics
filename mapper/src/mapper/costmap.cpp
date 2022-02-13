#include "mapper/costmap.h"
#include <cmath>

CostMap::CostMap (const double resolutionArg, const double min_xArg,
                  const double min_yArg, const unsigned int rowsArg,
                  const unsigned int colsArg) 
                  : resolution{resolutionArg}, min_x{min_xArg}, min_y{min_yArg}, rows{rowsArg},cols{colsArg}, cost_vector{std::vector<double>(cols * rows, 0)}
{}

// Returns the map's column index for the given x coordinate.
int CostMap::x_to_col(const double x) const {
  return floor((x - min_x) / resolution);
}

// Returns the map's row index for the given y coordinate.
int CostMap::y_to_row(const double y) const {
	return floor((y - min_y) / resolution);
}

// Returns the index in the grid of the given (x,y) point.
int CostMap::point_to_index(const double x, const double y) const {
	return cols * y_to_row(y) + x_to_col(x);
}

int CostMap::length() const {
	return cost_vector.size();
}

// Returns if the given (x,y) point is in the map's range.
bool CostMap::in_map(const double x, const double y) const {
	return min_x <= x && x <= min_x + cols * resolution 
		&& min_y <= y && y <= min_y + rows * resolution;
}

void CostMap::set(const double x, const double y, const double value) {
	cost_vector[y_to_row(y) * cols + x_to_col(x)] = value;
}

double CostMap::get(const double x, const double y) const {
	return cost_vector[y_to_row(y) * cols + x_to_col(x)];
}

std::string CostMap::to_string() const {
	std::string str = "";
	for(int i = 0; i < rows; i++) {
		str += "\t";

		for(int j = 0; j < cols; j++) 
			str += std::to_string(cost_vector[i * cols + j]) + " ";

		str += "\n";
	}

	return str;
}

std::ostream &operator<<(std::ostream &os, const CostMap &map) {
	return os << map.to_string();
}
