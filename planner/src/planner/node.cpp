// Author: Benned Hedegaard

#include "planner/node.h"

Node::Node( const int& xArg, const int& yArg ) {
    x = xArg;
    y = yArg;
    g = 0.0;
    f = 0.0;
    prev = nullptr;
}

Node::~Node() {} // Deconstructor
