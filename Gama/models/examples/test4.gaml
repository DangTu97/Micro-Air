/***
* Name: test4
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test4
import "../vehicle4.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */
global {
	file shape_file_roads <-  file("../includes/junction.shp");
	file shape_file_roadNodes <-  file("../includes/roadNode.shp");
	geometry shape <- envelope(shape_file_roads) + 20;
	float step <- STEP;
	init {
		create road from: shape_file_roads with:[is_twoway:: bool(read("type"))]{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
		}
		create roadNode from: shape_file_roadNodes;
		road_network <- directed(as_edge_graph(road));
	}
	
	reflex init_traffic when:mod(cycle,4)=0 {
		create vehicle number: 1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			max_speed <- 14 #m/#s;
			source_node <- one_of([roadNode(0).location, roadNode(3).location,roadNode(6).location,roadNode(8).location]);
			final_node <- one_of([roadNode(0).location, roadNode(3).location,roadNode(6).location,roadNode(8).location]);
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
