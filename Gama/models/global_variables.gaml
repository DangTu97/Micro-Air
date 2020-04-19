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
	
	float vehicle_range <- 15.0;
	float minimun_polygon_size <- 0.1;
	float polygon_width_size <- 0.5; // for the left and right polygon
	float acceleration_factor <- 0.05;
	float deceleration_factor <- 0.4;
	
	float time_to_change <- 200; // for traffic light
	float road_width <- 3;
	float distance_check <- 4; // distance condition to check whether a vehicle reach to next node
}