// Author: Benned Hedegaard

// Include the header we're defining methods for.
#include "package_name/package_name.h"

ClassName::ClassName() { // Constructor
	// Initialize anything that needs to be initialized.
}

ClassName::~ClassName() {} // Deconstructor

// Define all message handling functions.
// Simplest type: Just store the message locally.
void ClassName::handleMessageType( const package_name::DataType::ConstPtr& msg ) {
	_data = *msg;
	_data2 = msg->position.x; // Might want to access subparts of the message.
	
	// We could call other functions or publish new information if we wanted.
}

// Define any other methods in the header file.
void ClassName::step( double dt ) { // e.g. for a simulator
	// Process what's needed, update member variables, or publish things.
}

// Helper functions unimportant to the header can be defined as needed.
