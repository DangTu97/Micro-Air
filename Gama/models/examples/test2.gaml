/***
* Name: test
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test2
import "../vehicle4.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */
global {
	file shape_file_roads <-  file("../includes/test_road.shp");
	geometry shape <- envelope(shape_file_roads) + 2*ROAD_WIDTH;

	float step <- STEP;
	init {
		create road  from: shape_file_roads with:[is_twoway: bool(read('twoway'))]{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
		}
		road_network <- directed(as_edge_graph(road));
		
		create vehicle number:100 {
			type <- 'CAR';
			source_node <- one_of(road_network.vertices);
			final_node <- one_of(road_network.vertices);
			do set_type;
		}
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background:#grey{
			species road aspect:base;
			species vehicle aspect:base;
		}
	}
}