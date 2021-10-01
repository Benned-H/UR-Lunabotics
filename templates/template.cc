/**
 * An example source file implementing the functions for some class
 * Author: Benned Hedegaard
 */

// Include the header we're defining methods for.
#include "package_name/class-name.h"

//
// Constructor for the ClassName class
//
ClassName::ClassName(){
  // Initialize anything that needs to be initialized using the initializer list
}

//
// This method handles a DataType message
//
void ClassName::handleMessageType( const package_name::DataType::ConstPtr& msg ){
  // Simplest type of message handler: Just store the message locally.
  data = *msg;
  data2 = msg->position.x; // Might want to access subparts of the message.
	
  // We could call other functions or publish new information if we wanted.
}

//
// Define any other methods in the header file. Put the @brief description for each method above it in a comment like this
//
void ClassName::step( double& dt ){ // e.g. for a simulator
  // Process what's needed, update member variables, or publish things.
}

// Disclaimer: Generally it's best to keep helper functions rare. That is, all functions should show up in the header file.
// If the functionality is common, put it into a shared common namespace.
