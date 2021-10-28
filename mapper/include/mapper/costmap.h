// Author: Benned Hedegaard

#pragma once

#include <vector>

class CostMap {
// TODO - Create separate OccupancyGrid class to clean up which functions are where
// TODO - Rename this class to something clearer

public:
  /*
    OccMapper class constructor
    TODO - Redo this documentation using Doxygen

    resolutionArg - Resolution (m) of the cost map
    mapWidthArg - Width (number of cells) of the cost map
    mapHeightArg - Height (number of cells) of the cost map
    origin - Origin of the cost map
    obstacleWidthArg - Presumed width (m) of a detected obstacle
    thresholdArg - Threshold of occupancy over which a cell is deemed occupied
    p0, p_free, p_occ - Probability parameters of the occupancy grid mapping algorithm
  */
  CostMap(const double, const double, 
				const double, const unsigned int, 
				const unsigned int);
		
  virtual ~CostMap() = default; // Default deconstructor

  /**
   * Converts some x coordinate in the global frame to a column in the costmap.
   *
   * @brief Converts some x coordinate in the global frame to a column in the costmap.
   * @param[in]		x	x value to convert
   * @returns		corresponding column as an int
   */
  int x_to_col(const double);
  
  /**
   * Converts some y coordinate in the global frame to a row in the costmap.
   *
   * @brief Converts some y coordinate in the global frame to a row in the costmap.
   * @param[in]		y	y value to convert
   * @returns		corresponding column as an int
   */
  int y_to_row(const double);

  /**
   * TODO
   */ 
  int point_to_index(const double, const double);

  int length();

  bool in_map(const double x, const double);

  /**
   * Set TODO
   */
  void set(const int, const int, const double);

  /**
   * Get TODO
   */
  double get(const int, const int);

  std::string to_string() const;

protected:
private:
  /* The size (m) of the side of each cell in the costmap */
  double resolution;

  /* Minimum x coordinate contained in the costmap */
  double min_x;

  /* Minimum y coordinate contained in the costmap */
  double min_y;

  /* Number of rows in the costmap */
  int rows;

  /* Number of columns in the costmap */
  int cols;

  std::vector<double> cost_vector;
};
/* Ex:           col#
/*      {     0  1  2  3
          0  {1, 2, 3, 4},
    row#  1  {5, 6, 7, 8}
          2  {9, 10, 11, 12}
        }
        |
        |
        -> {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}
*/

std::ostream &operator<<(std::ostream &os, const CostMap &map);
