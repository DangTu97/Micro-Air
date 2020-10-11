/***
* Name: addfield
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model addfield
import "../vehicle4.gaml"
/* Insert your model definition here */

global {
	file shape_file_roads <-  file("../includes/road.shp");
	init {
		create road from: shape_file_roads;
		save road type:'shp' to:"test_road.shp" attributes:["twoway"::true];
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background:#grey{
			species road aspect:base;
		}
	}
}