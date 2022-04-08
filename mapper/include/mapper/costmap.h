#ifndef COSTMAP_H
#define COSTMAP_H

#include <vector>
#include <string>
#include "nav_msgs/OccupancyGrid.h"


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

public:
  /**
   * @brief CostMap class constructor
   * 
   * @param resolutionArg The size (m) of the side of each cell
   * @param min_xArg Minimum x coordinate
   * @param min_yArg Minimum y coordinate
   * @param rowsArg Number of rows 
   * @param colsArg Number of columns
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
   * @brief Returns the index in the grid of the given (x,y) point.
   */
  int point_to_index(const double x, const double y) const;

  /**
   * @brief Returns the size of this costmap
   */
  int length() const;

  /**
   * @brief Returns if the given (x,y) point is in the map's range.
   */
  bool in_map(const double x, const double y) const;

  void set(const double x, const double y, const uint8_t value);

  int8_t get(const double x, const double y) const;

  std::string to_string () const;


  std::vector<int8_t> cost_vector;
};

std::ostream &operator<< (std::ostream &os, const CostMap &map);


#endif //COSTMAP_H