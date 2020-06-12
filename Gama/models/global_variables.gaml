/***
* Name: globalvariables
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model globalvariables

/* Insert your model definition here */

global {
	float STEP <- 0.05#s;
	float environment_size <- 300.0;
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png"), image_file("../includes/bus.jpg")];
	
	float vehicle_range <- 20.0;
	float minimun_polygon_size <- 0.15;
	float polygon_width_size <- 0.5; // for the left and right polygon
	float acceleration_factor <- 0.5;
	float deceleration_factor <- 5.0;
	
	float green_time <- 400; // for traffic light
	float yellow_time <- 80;
	float red_time <- 480; // red time = green time + yellow time
//	float red_time <- 750; 
	float road_width <- 8.0;
	float distance_check <- 20.0; // distance condition to check whether a vehicle reach to next node
	float prob_pass_light <- 0.0;
	
	// vehicle size
	float CAR_LENGTH <- 3.8 #m;
	float CAR_WIDTH <- 1.5 #m;
	float CAR_DF <- 0.15 #m; //safe distance for vehicle in front
	float CAR_DB <- 0.1 #m; //safe distance for vehicle beside
	
	float MOTORBIKE_LENGTH <- 1.9 #m;
	float MOTORBIKE_WIDTH <- 0.7 #m;
	float MOTORBIKE_DF <- 0.1 #m; //safe distance for vehicle in front
	float MOTORBIKE_DB <- 0.05 #m; //safe distance for vehicle beside
	
	float BUS_LENGTH <- 9.45 #m;
	float BUS_WIDTH <- 2.45 #m;
	float BUS_DF <- 0.4 #m; //safe distance for vehicle in front
	float BUS_DB <- 0.2 #m; //safe distance for vehicle beside
	float BUS_MAXSPEED <- 14 #m/#s;
	
	float INIT_SPEED <- 8 #m/#s;
	float PROB_GO_OPPOSITE <- 0.0;
	float PROB_TURN_RIGHT <- 1.0;
	float TIME_STEP <- 0.05 #s;
	
	//Vietnamese traffic information
	float CAR_PERCENT <- 0.2;
	
	float MOTORBIKE_SAFESPEED_YOUNG_MALE <- 9.7 #m/#s;
	float MOTORBIKE_SAFESPEED_MIDDLEAGED_MALE <- 8.6 #m/#s;
	float MOTORBIKE_SAFESPEED_OLD_MALE <- 6.9 #m/#s;
	float MOTORBIKE_SAFESPEED_YOUNG_FEMALE <- 8.9 #m/#s;
	float MOTORBIKE_SAFESPEED_MIDDLEAGED_FEMALE <- 7.8 #m/#s;
	float MOTORBIKE_SAFESPEED_OLD_FEMALE <- 6.1 #m/#s;
	
	// maximum allowed speed by law for motorbike
	float MOTORBIKE_MAXSPEED_YOUNG_MALE <- 11.1 #m/#s;
	float MOTORBIKE_MAXSPEED_MIDDLEAGED_MALE <- 10.6 #m/#s;
	float MOTORBIKE_MAXSPEED_OLD_MALE <- 9.7 #m/#s;
	float MOTORBIKE_MAXSPEED_YOUNG_FEMALE <- 11.0 #m/#s;
	float MOTORBIKE_MAXSPEED_MIDDLEAGED_FEMALE <- 9.0 #m/#s;
	float MOTORBIKE_MAXSPEED_OLD_FEMALE <- 8.0 #m/#s;
	
	float MOTORBIKE_ACCELERATION_YOUNG_MALE <- 11 #m/#s;
	float MOTORBIKE_ACCELERATION_MIDDLEAGED_MALE <- 11 #m/#s;
	float MOTORBIKE_ACCELERATION_OLD_MALE <- 11 #m/#s;
	float MOTORBIKE_ACCELERATION_YOUNG_FEMALE <- 11 #m/#s;
	float MOTORBIKE_ACCELERATION_MIDDLEAGED_FEMALE <- 11 #m/#s;
	float MOTORBIKE_ACCELERATION_OLD_FEMALE <- 11 #m/#s;
	
	float CAR_SAFESPEED_YOUNG_MALE <- 11.1 #m/#s;
	float CAR_SAFESPEED_MIDDLEAGED_MALE <- 10 #m/#s;
	float CAR_SAFESPEED_OLD_MALE <- 9.2 #m/#s;
	float CAR_SAFESPEED_YOUNG_FEMALE <- 10.6 #m/#s;
	float CAR_SAFESPEED_MIDDLEAGED_FEMALE <- 9.7 #m/#s;
	float CAR_SAFESPEED_OLD_FEMALE <- 8.8 #m/#s;
	
	// maximum allowed speed by law for car
	float CAR_MAXSPEED_YOUNG_MALE <- 13.8 #m/#s;
	float CAR_MAXSPEED_MIDDLEAGED_MALE <- 12.5 #m/#s;
	float CAR_MAXSPEED_OLD_MALE <- 11.1 #m/#s;
	float CAR_MAXSPEED_YOUNG_FEMALE <- 12.5 #m/#s;
	float CAR_MAXSPEED_MIDDLEAGED_FEMALE <- 11.9 #m/#s;
	float CAR_MAXSPEED_OLD_FEMALE <- 10.3 #m/#s;
	
	float CAR_ACCELERATION_YOUNG_MALE <- 11 #m/#s;
	float CAR_ACCELERATION_MIDDLEAGED_MALE <- 11 #m/#s;
	float CAR_ACCELERATION_OLD_MALE <- 11 #m/#s;
	float CAR_ACCELERATION_YOUNG_FEMALE <- 11 #m/#s;
	float CAR_ACCELERATION_MIDDLEAGED_FEMALE <- 11 #m/#s;
	float CAR_ACCELERATION_OLD_FEMALE <- 11 #m/#s;
	
	// probability to choose maximum allowed speed as max_speed
	float YOUNGLE_MALE_PROB <- 0.7;
	float MIDDLEAGED_MALE_PROB <- 0.5;
	float OLD_MALE_PROB <- 0.4;
	
	float YOUNGLE_FEMALE_PROB <- 0.6;
	float MIDDLEAGED_FEMALE_PROB <- 0.48;
	float OLD_FEMALE_PROB <- 0.35;
	
	//parameter
	float CAR_MAXSPEED <- 14 #m/#s;
	float CAR_SAFESPEED_MALE <- 10 #m/#s;
	float CAR_SAFESPEED_FEMALE <- 8.9 #m/#s;
	
	float MOTORBIKE_MAXSPEED <- 11 #m/#s;
	float MOTORBIKE_SAFESPEED_MALE <- 8.3 #m/#s;
	float MOTORBIKE_SAFESPEED_FEMALE <- 7.5 #m/#s;
	
	float PROB_MAXSPEED_SAFESPEED <- 0.5;
}