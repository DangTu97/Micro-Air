/**
* Name: globalvars
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model globalvars

/* Insert your model definition here */

global {
	//vehicle
	//motorbike
	float MOTORBIKE_LENGTH <- 1.9#m;
	float MOTORBIKE_WIDTH <- 0.7#m;
	float MOTORBIKE_DF <- 0.1#m;  
	float MOTORBIKE_DB <- 0.05#m; 
	float MOTORBIKE_MAXSPEED <- 11#m/#s;
	image_file MOTORBIKE_IMAGE <- image_file("../../includes/motorbike.png");
	float MOTORBIKE_ACCELERATION_RATE <- 2.0#m/#s;
	float MOTORBIKE_DECELERATION_RATE <- 4.0#m/#s;
	
	//car
	float CAR_LENGTH <- 4.0#m;
	float CAR_WIDTH <- 2.0#m;
	float CAR_DF <- 0.15#m; 
	float CAR_DB <- 0.1#m; 
	float CAR_MAXSPEED <- 14 #m/#s;
	image_file CAR_IMAGE <- image_file("../../includes/car.png");
	float CAR_ACCELERATION_RATE <- 0.5#m/#s;
	float CAR_DECELERATION_RATE <- 4.0#m/#s;
	
	//driver
	float COLLISION_AVOIDANCE_DURATION <- 2.0#s;
	float REACTION_TIME <- 0.5#s;
	
	//environment config
	float STEP <- 0.2#s;
	float MINIMUM_DURATION <- 0.05#s;
	
}