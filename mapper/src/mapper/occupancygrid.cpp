#include "mapper/occupancygrid.h"

OccupancyGrid::OccupancyGrid(const double &resolutionArg, 
                              const unsigned int &mapWidthArg, const unsigned int &mapHeightArg, 
                              const geometry_msgs::Pose &origin, 
                              const double &obstacleWidthArg, const int &thresholdArg, 
                              const double &p0, const double &p_free, const double &p_occ) : 
  costmap{resolutionArg == 0.0 ? 0.1 : resolutionArg, origin.position.x, origin.position.y, mapHeightArg, mapWidthArg},

  threshold{thresholdArg},

  occ_steps{(int)std::floor(obstacleWidthArg / RESOLUTION)},

  l0{log_odds(p0)},

  l_free{log_odds(p_free)},

  l_occ{log_odds(p_occ)}
{};

void OccupancyGrid::set_map(const nav_msgs::OccupancyGrid occ) {
  costmap.cost_vector = occ.data;
}
