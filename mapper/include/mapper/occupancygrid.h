#pragma once

#include "costmap.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

class OccupancyGrid {
public:
  /**
   * @brief OccMapper class constructor
   * 
   * @param resolutionArg Resolution (m) of the cost map
   * @param mapWidthArg Width (number of cells) of the cost map
   * @param mapHeightArg Height (number of cells) of the cost map
   * @param origin Origin of the cost map
   * @param obstacleWidthArg Presumed width (m) of a detected obstacle
   * @param thresholdArg Threshold of occupancy over which a cell is deemed occupied
   * @param p0 Probability parameters of the occupancy grid mapping algorithm
   * @param p_free same as p0
   * @param p_occ same as p0
   */
  OccupancyGrid( const double& resolutionArg, 
                  const unsigned int& mapWidthArg, const unsigned int& mapHeightArg, 
                  const geometry_msgs::Pose& origin, 
                  const double& obstacleWidthArg, const int& thresholdArg, 
                  const double& p0, const double& p_free, const double& p_occ );

  void set_map(const nav_msgs::OccupancyGrid occ);

  bool occupied(double x, double y);

protected:

  geometry_msgs::Pose pose;
  int occ_steps; // Depth of obstacles in steps of size RESOLUTION.
  int threshold; // We consider cells with this value or greater occupied.

  double l0; // Prior log-odds of occupancy.
  double l_occ; // Log-odds of occupied cell given laser hit.
  double l_free; // Log-odds of free cell given laser passes through.
		
  // Map parameters
  CostMap costmap;
  double RESOLUTION;
  double MIN_X;
  double MAX_X;
  double MIN_Y;
  double MAX_Y;


private:
  // Formula from Wikipedia for now. More quaternion understanding is needed.
  // TODO - Move to common
  constexpr double quat_to_yaw( const geometry_msgs::Quaternion& q ) {
    return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  }

  // Returns log-odds representation of the given probability.
  constexpr double log_odds( const double& probability ) {
    return std::log( probability / (1.0 - probability) );
  }
};
