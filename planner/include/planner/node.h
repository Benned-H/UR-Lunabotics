// Author: Benned Hedegaard

#ifndef NODE_H
#define NODE_H

#include <memory>

class Node {

    public:
    
	    Node( const int& xArg, const int& yArg );
	    virtual ~Node(); // Deconstructor
	    
	    int x; // Discrete locations in the grid; (0,0) is the origin
	    int y;
	    double g; // Cost-so-far during heuristic search
	    double f; // Estimate of total cost during search; f = g + h
	    std::shared_ptr<Node> prev; // Backpointer to previous Node
};

#endif /* NODE_H */
