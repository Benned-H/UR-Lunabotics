// Author: Benned Hedegaard

#pragma once

#include <memory>

class State{
public:

  /**
   * Constructor for State class
   *
   * @brief Constructor for State class
   * @param[in]   xArg        x coordinate
   * @param[in]   yArg        y coordinate
   * @param[in]   thetaArg    angle about robot z-axis
   * @param[in]   vxArg       linear velocity along robot x-axis
   * @param[in]   wzArg       angular velocity about robot z-axis
   */
  State( double xArg, double yArg, double thetaArg, double vxArg, double wzArg );

  /**
   * Default deconstructor
   */
  virtual ~State() = default;

  /** Continuous x coordinate */
  double x;
  /** Continuous y coordinate */
  double y;
  /** Robot angle about z-axis */
  double theta;
  /** Linear velocity along robot x-axis */
  double v_x;
  /** Angular velocity about robot z-axis */
  double w_z;
};
