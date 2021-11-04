// Author: Benned Hedegaard

#include <cmath>
#include <iostream>
#include <vector>

#include "mapper/costmap.h"

/*
	OccMapper class constructor

	resolutionArg - Resolution (m) of the cost map
	mapWidthArg - Width (number of cells) of the cost map
	mapHeightArg - Height (number of cells) of the cost map
	origin - Origin of the cost map
	obstacleWidthArg - Presumed width (m) of a detected obstacle
	thresholdArg - Threshold of occupancy over which a cell is deemed occupied
	p0, p_free, p_occ - Prior, free, and occupied probabilities of cells given laser hit
*/
CostMap::CostMap(const double resolutionArg, const double min_xArg, 
				const double min_yArg, const unsigned int rowsArg, 
				const unsigned int colsArg) {
  // TODO - Correctly initialize all class member variables
	
	resolution = resolutionArg;

	min_x = min_xArg;
	min_y = min_yArg;

	rows = rowsArg;
	cols = colsArg;

	cost_vector = std::vector<double>(rows * cols);
	//can remodify if you want to pass in arguments for these values in advanced
}

// Returns the map's column index for the given x coordinate.
int CostMap::x_to_col(const double x) {
	return floor((x - min_x) / resolution);
}

// Returns the map's row index for the given y coordinate.
int CostMap::y_to_row(const double y) {
		return floor((y - min_y) / resolution);
}

// Returns the index in the grid of the given (x,y) point.
int CostMap::point_to_index(const double x, const double y) {
	return cols * y_to_row(y) + x_to_col(x);
}

int CostMap::length() {
	return cost_vector.size();
}

// Returns if the given (x,y) point is in the map's range.
bool CostMap::in_map(const double x, const double y) {
	return min_x <= x && x <= min_x + cols * resolution 
		&& min_y <= y && y <= min_y + rows * resolution;
}

void CostMap::set(const double x, const double y, const double value) {
	cost_vector[y_to_row(y) * cols + x_to_col(x)] = value;
}

double CostMap::get(const double x, const double y) {
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