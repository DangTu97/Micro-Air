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
}