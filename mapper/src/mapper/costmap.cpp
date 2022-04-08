#include "mapper/costmap.h"
#include <cmath>

CostMap::CostMap (const double resolutionArg, const double min_xArg,
                  const double min_yArg, const unsigned int rowsArg,
                  const unsigned int colsArg) 
                  : resolution{resolutionArg}, min_x{min_xArg}, min_y{min_yArg}, rows{rowsArg},cols{colsArg}, cost_vector(cols * rows, 0)
{}

int CostMap::x_to_col(const double x) const {
  return std::floor((x - min_x) / resolution);
}

int CostMap::y_to_row(const double y) const {
	return std::floor((y - min_y) / resolution);
}

int CostMap::point_to_index(const double x, const double y) const {
	return cols * y_to_row(y) + x_to_col(x);
}

int CostMap::length() const {
	return cost_vector.size();
}

bool CostMap::in_map(const double x, const double y) const {
	return min_x <= x && x <= min_x + cols * resolution 
		&& min_y <= y && y <= min_y + rows * resolution;
}

void CostMap::set(const double x, const double y, const uint8_t value) {
	cost_vector[y_to_row(y) * cols + x_to_col(x)] = value;
}

int8_t CostMap::get(const double x, const double y) const {
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
