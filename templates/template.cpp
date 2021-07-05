// Author: Benned Hedegaard

// Include the header we're defining methods for.
#include "package_name/package_name.h"

ClassName::ClassName(){ // Constructor
  // Initialize anything that needs to be initialized using the initializer list
}

ClassName::~ClassName() {} // Deconstructor

// Define all message handling functions.
// Simplest type: Just store the message locally.
void ClassName::handleMessageType( const package_name::DataType::ConstPtr& msg ){
  data = *msg;
  data2 = msg->position.x; // Might want to access subparts of the message.
	
  // We could call other functions or publish new information if we wanted.
}

// Define any other methods in the header file.
void ClassName::step( double dt ){ // e.g. for a simulator
  // Process what's needed, update member variables, or publish things.
}

// Disclaimer: Generally it's best to keep helper functions rare.
// If the functionality is common, put it into a shared common namespace.
