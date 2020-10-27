/***
* Name: test
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test
import "../vehicle4.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */
global {
	file shape_file_roads <-  file("../includes/multilines.shp");
	geometry shape <- envelope(shape_file_roads) + 2*ROAD_WIDTH;
	float step <- STEP;
	init {
		create road from: shape_file_roads{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
	}
	
	reflex init_traffic when:mod(cycle,10)=0 {
		create vehicle number:1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			source_node <- road_network.vertices[1];
			final_node <- road_network.vertices[0];
			do set_type;
		}
		
		create vehicle number:1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
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