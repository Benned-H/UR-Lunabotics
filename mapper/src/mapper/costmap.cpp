// Author: Benned Hedegaard

#include <cmath>
#include <set>
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

	for(int i = 0; i < rows * cols; i++)
		cost_vector.emplace_back(0.0); 
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
	return rows * y_to_row(y) + x_to_col(x);
}

// Returns if the given (x,y) point is in the map's range.
bool CostMap::in_map(const double x, const double y) {
	return min_x <= x && x <= min_x + cols * resolution 
		&& min_y <= y && y <= min_y + rows * resolution;
}

void CostMap::set(const int col, const int row, const double value) {
	cost_vector[col * cols + row] = value;
}

double CostMap::get(const int col, const int row) {
	return cost_vector[col * cols + row];
}
