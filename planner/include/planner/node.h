// Author: Benned Hedegaard

#pragma once

#include <memory>

class Node{
public:

  /**
   * Constructor for Node class
   *
   * @brief Constructor for Node class
   * @param[in]   xArg        discrete x coordinate
   * @param[in]   yArg        discrete y coordinate
   * @param[in]   thetaArg    discrete robot angle (0 to 7)
   * @param[in]   vxArg       discrete linear velocity (-1 to 1)
   */
  Node( int xArg, int yArg, int thetaArg, int vxArg );

  /**
   * Default deconstructor
   */
  virtual ~Node() = default;

  /** Discrete x coordinate */
  int i_x;
  /** Discrete y coordinate */
  int i_y;
  /** Discrete angle (0 to 7) representing steps of 45 degrees */
  int i_theta;
  /** Discrete linear velocity (v_x) either -1, 0, or 1 (multiply by 0.25 m/s) */
  int i_vx;
  // We assume all nodes have zero angular velocity (w_z)

  /** Cost-so-far during heuristic search */
  double g;
  /** Estimate of total cost during search: f = g + h */
  double f;
  /** Backpointer to previous Node */
  std::shared_ptr<Node> prev;
};
