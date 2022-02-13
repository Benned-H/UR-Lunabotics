#ifndef COSTMAP_H
#define COSTMAP_H

#include <vector>
#include <string>
#include <iostream>

class CostMap
{
private:
  /* The size (m) of the side of each cell in the costmap */
  double resolution;

  /* Minimum x coordinate contained in the costmap */
  double min_x;

  /* Minimum y coordinate contained in the costmap */
  double min_y;

  /* Number of rows in the costmap */
  unsigned int rows;

  /* Number of columns in the costmap */
  unsigned int cols;

  std::vector<double> cost_vector;

public:
  /**
   * OccMapper class constructor
   * TODO - Redo this documentation using Doxygen

   * resolutionArg - Resolution (m) of the cost map
   * mapWidthArg - Width (number of cells) of the cost map
   * mapHeightArg - Height (number of cells) of the cost map
   * origin - Origin of the cost map
   * obstacleWidthArg - Presumed width (m) of a detected obstacle
   * thresholdArg - Threshold of occupancy over which a cell is deemed occupied
   * p0, p_free, p_occ - Probability parameters of the occupancy grid mapping algorithm
   */
  CostMap (const double resolutionArg, const double min_xArg,
              const double min_yArg, const unsigned int rowsArg,
              const unsigned int colsArg);

  /**
   * Converts some x coordinate in the global frame to a column in the costmap.
   *
   * @brief Converts some x coordinate in the global frame to a column in the costmap.
   * @param in		x	x value to convert
   * @returns		corresponding column as an int
   */
  int x_to_col(const double x) const;
  
  /**
   * Converts some y coordinate in the global frame to a row in the costmap.
   *
   * @brief Converts some y coordinate in the global frame to a row in the costmap.
   * @param in		y	y value to convert
   * @returns		corresponding column as an int
   */
  int y_to_row(const double y) const;

  /**
   * TODO
   */ 
  int point_to_index(const double x, const double y) const;

  int length() const;

  bool in_map(const double x, const double y) const;

  void set(const double x, const double y, const double value);

  double get(const double x, const double y) const;

  std::string to_string () const;

};

std::ostream &operator<< (std::ostream &os, const CostMap &map);


#endif //COSTMAP_H
