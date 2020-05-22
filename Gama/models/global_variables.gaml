/***
* Name: globalvariables
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model globalvariables

/* Insert your model definition here */

global {
	float environment_size <- 200.0;
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png")];
	
	float vehicle_range <- 20.0;
	float minimun_polygon_size <- 0.1;
	float polygon_width_size <- 0.5; // for the left and right polygon
	float acceleration_factor <- 0.1;
	float deceleration_factor <- 0.4;
	
	float green_time <- 400; // for traffic light
	float yellow_time <- 80;
	float red_time <- 480; // red time = green time + yellow time
//	float red_time <- 750; 
	float road_width <- 8.0;
	float distance_check <- 20.0; // distance condition to check whether a vehicle reach to next node
	float prob_pass_light <- 0.0;
	
	// vehicle
	float CAR_LENGTH <- 3.8 #m;
	float CAR_WIDTH <- 1.5 #m;
	float CAR_DF <- 0.15 #m; //safe distance for vehicle in front
	float CAR_DB <- 0.05 #m; //safe distance for vehicle beside
	float CAR_MAXSPEED <- 0.7 #m/#s;
	
	float MOTORBIKE_LENGTH <- 1.9 #m;
	float MOTORBIKE_WIDTH <- 0.7 #m;
	float MOTORBIKE_DF <- 0.01 #m; //safe distance for vehicle in front
	float MOTORBIKE_DB <- 0.05 #m; //safe distance for vehicle beside
	float MOTORBIKE_MAXSPEED <- 0.55 #m/#s;
	
	float INIT_SPEED <- 0.3;
	float PROB_GO_OPPOSITE <- 0.0;
	float PROB_TURN_RIGHT <- 1.0;
	float TIME_STEP <- 0.05 #s;
}