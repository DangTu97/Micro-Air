/***
* Name: test3
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model test3
import "../vehicle4.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */

global {
	file shape_file_roads <-  file("../includes/road_circle.shp");
	geometry shape <- envelope(shape_file_roads) + 20;
	float step <- STEP;
	init {
		create road from: shape_file_roads with:[is_twoway:: bool(read("type"))]{
			width <- ROAD_WIDTH;
			geom_display <- shape + width;
		}
		road_network <- directed(as_edge_graph(road));
		write road_network;
	}
	
	reflex init_traffic when:mod(cycle,5)=0 {
		create vehicle number: 1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			max_speed <- 14 #m/#s;
			source_node <- {20,60};
			final_node <- {140,20};
			do set_type;
//			shortest_path <- [road(0), road(2), road(3), road(1)];
		}
		
		create vehicle number: 1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			max_speed <- 14 #m/#s;
			source_node <- {140,20};
			final_node <- {20,60};
			do set_type;
//			shortest_path <- [road(1), road(5), road(4), road(0)];
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