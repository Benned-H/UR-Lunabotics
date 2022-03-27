// Author: Benned Hedegaard

#include "planner/node.h"

//
// Constructor for Node class
//
Node::Node( int xArg, int yArg, int thetaArg, int vxArg ) :
  i_x( xArg ), i_y( yArg ), i_theta( thetaArg ), i_vx( vxArg ), g( 0.0 ), f( 0.0 ), prev( nullptr ){

}
