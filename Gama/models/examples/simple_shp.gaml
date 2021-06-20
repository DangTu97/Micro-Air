/**
* Name: simpleshp
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model simpleshp
import '../species/vehicle.gaml'
import '../species/GIS_species.gaml'
import '../global_vars.gaml'
/* Insert your model definition here */

global {
	file shape_file_roads <- file("../includes/multilines.shp");
	geometry shape <- envelope(shape_file_roads) + 10;
	float step <- STEP;
	init {
		create road from:shape_file_roads {
			width <- 5#m;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
		write road_network;
		loop v over: road_network.vertices {
			create roadNode {
				location <- v;
				is_intersection <- false;
				geom_display <- shape + 1;
			}
		}
		road(0).endpoints <- [roadNode(0), roadNode(1)];
		create vehicle number:1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			source_node <- road_network.vertices[1];
			destination_node <- road_network.vertices[0];
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}
	
	reflex init_traffic when:mod(cycle,4)=0 {
		create vehicle number:1 {
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			source_node <- road_network.vertices[1];
			destination_node <- road_network.vertices[0];
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
		
		create vehicle number:1 {
			source_node <- road_network.vertices[0];
			destination_node <- road_network.vertices[1];
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}	
}

experiment test_shp {
	float minimum_cycle_duration <- MINIMUM_DURATION;
	output {
		display my_display background:#grey {
			species road aspect:base;
			species roadNode aspect:base;
			species vehicle aspect:base;
		}
	}
}