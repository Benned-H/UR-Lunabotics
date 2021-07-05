// Author: Benned Hedegaard

#include <cmath>

#include "planner/gridplanner.h"

GridPlanner::GridPlanner( const double& discretization, const OccMapper& mapArg ) : cost_map( mapArg ) {
	DISCRETIZATION = discretization;
}

GridPlanner::~GridPlanner() {} // Deconstructor

void GridPlanner::handleQuery( const planner::Query::ConstPtr& msg ) {
	planner::Path p;
	p.points = aStar( msg->start, msg->goal );
	path_pub.publish(p);
}

void GridPlanner::handleMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    cost_map._map = *msg;
}

// TODO - Common package/file
double euclidean( const double& x1, const double& y1, const double& x2, const double& y2 ) {
	return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

// Computes real-world distance between discretized integer points.
// TODO - Common package
double discrete_euclidean( const int& x1, const int& y1, const int& x2, const int& y2, const int& d ) {
	return d * sqrt( (double)((x1-x2)*(x1-x2)) + (double)((y1-y2)*(y1-y2)) );
}

// Sorts nodes in ascending order by their f value.
bool compNodes( std::shared_ptr<Node>& n1, std::shared_ptr<Node>& n2 ) {
	return (n1->f < n2->f);
}

// Generates 8-connected neighbors of given Node.
std::vector< std::shared_ptr<Node> > getNeighbors( std::shared_ptr<Node>& curr ) {
    std::vector< std::shared_ptr<Node> > neighbors;
    
    neighbors.push_back( std::make_shared<Node>( curr->x+1, curr->y ) );
    neighbors.push_back( std::make_shared<Node>( curr->x+1, curr->y+1 ) );
    neighbors.push_back( std::make_shared<Node>( curr->x, curr->y+1 ) );
    neighbors.push_back( std::make_shared<Node>( curr->x-1, curr->y+1 ) );
    neighbors.push_back( std::make_shared<Node>( curr->x-1, curr->y ) );
    neighbors.push_back( std::make_shared<Node>( curr->x-1, curr->y-1 ) );
    neighbors.push_back( std::make_shared<Node>( curr->x, curr->y-1 ) );
    neighbors.push_back( std::make_shared<Node>( curr->x+1, curr->y-1 ) );
    
    return neighbors;
}

// Runs A* on a grid with the stored discretization.
std::vector<geometry_msgs::Point> GridPlanner::aStar( const geometry_msgs::Point& start, const geometry_msgs::Point& goal ) {
	// First create the open and closed lists.
	std::vector< std::shared_ptr<Node> > open_list; // TODO - Better data structure (e.g. minheap) for open list!
	std::vector< std::shared_ptr<Node> > closed_list;

	// Start from the closest on-grid point.
	std::shared_ptr<Node> s0 = std::make_shared<Node>( std::round(start.x / DISCRETIZATION), std::round(start.y / DISCRETIZATION) );
	s0->g = 0.0;
	s0->f = euclidean( s0->x, s0->y, goal.x, goal.y );
	open_list.push_back( s0 );
	
	std_msgs::UInt32 open_list_size; // For debugging purposes.
	std_msgs::UInt32 closed_list_size;
	
	// Main loop as long as the open list contains nodes.
	while ( open_list.size() > 0 ) {
	
	    // Publish open/closed list sizes
		open_list_size.data = open_list.size();
		closed_list_size.data = closed_list.size();
		open_list_size_pub.publish( open_list_size );
		closed_list_size_pub.publish( closed_list_size );
	
		// Pop best node, put into closed list.
		std::shared_ptr<Node> curr = open_list.front();
		open_list.erase( open_list.begin() );
		closed_list.push_back( curr );

		// If we've almost reached the goal, extract and return the path.
		if ( euclidean( (curr->x)*DISCRETIZATION, (curr->y)*DISCRETIZATION, goal.x, goal.y ) <= DISCRETIZATION ) {
			std::vector<geometry_msgs::Point> path;
			if ( ( DISCRETIZATION*curr->x != goal.x ) || ( DISCRETIZATION*curr->y != goal.y ) ) {
				path.push_back( goal ); // Include goal as final path point if Node didn't precisely reach it.
			}
			geometry_msgs::Point p;
			p.x = DISCRETIZATION*curr->x;
			p.y = DISCRETIZATION*curr->y;
			path.insert( path.begin(), p );

			while ( curr->prev != nullptr ) {
				curr = curr->prev;
				geometry_msgs::Point p;
				p.x = DISCRETIZATION*curr->x;
			    p.y = DISCRETIZATION*curr->y;
				path.insert( path.begin(), p );
			}

			return path;
		}

		// Consider the valid actions from the current node.
		std::vector< std::shared_ptr<Node> > neighbors = getNeighbors( curr );

		for ( int i = 0; i < neighbors.size(); i++ ) {
			auto new_node = neighbors[i];
			
			if ( cost_map.occupied( DISCRETIZATION*new_node->x, DISCRETIZATION*new_node->y ) ) {
				std::cout << "Node was occupied: (" << (DISCRETIZATION*new_node->x) << "," << (DISCRETIZATION*new_node->y) << ")" << std::endl;
				continue;
			}
			
			// Check if the neighbor is already closed
			bool closed = false;
			for ( int c = 0; c < closed_list.size(); c++ ) {
			    // TODO - Node equality function on discrete indices
				if ( closed_list[c]->x == new_node->x && closed_list[c]->y == new_node->y ) {
					closed = true;
					break;
				}
			}

			if ( closed ) { // Don't expand already-closed nodes.
				continue;
			}

			// Set new node's g and f values and its previous node.
			new_node->g = curr->g + discrete_euclidean( new_node->x, new_node->y, curr->x, curr->y, DISCRETIZATION );
			new_node->f = new_node->g + euclidean( DISCRETIZATION*new_node->x, DISCRETIZATION*new_node->y, goal.x, goal.y );
			new_node->prev = curr;

			// Push to the open list.
			open_list.push_back( new_node );
		}
		sort( open_list.begin(), open_list.end(), compNodes );
	} // end while

	// If the main while loop failed, we've lost. Return empty list.
	std::vector<geometry_msgs::Point> output;
	return output;
}

