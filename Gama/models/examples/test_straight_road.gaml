/**
* Name: teststraightroad
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model teststraightroad
import '../species/vehicle.gaml'
import '../species/GIS_species.gaml'
import '../global_vars.gaml'

/* Insert your model definition here */

global {
	geometry shape <- square(250);
	float step <- STEP;
	graph road_network;
	init {
		create road {
			shape <- polyline([{10, 150}, {200, 150}]);
			width <- 8.0#m;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
		write road_network;
		loop v over: road_network.vertices {
			create roadNode {
				location <- v;
				is_intersection <- false;
			}
		}
		loop v over: road_network.vertices {
			create roadNode {
				location <- v;
				is_intersection <- false;
				geom_display <- shape + 1;
			}
		}
		road(0).endpoints <- [roadNode(0), roadNode(1)];
		create vehicle number: 1 {	
			source_node <- road_network.vertices[0];
			destination_node <- road_network.vertices[1];
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			do init_type;
			location <- get_orthogonal_point(start_node, target_node, distance_to(start_node, target_node), rnd(dx, width_of_road_belong - dx));
			speed <- rnd(5.0, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}
	
	reflex init_traffic when:mod(cycle,4) = 0 {
		create vehicle number: 1 {	
			source_node <- road_network.vertices[0];
			destination_node <- road_network.vertices[1];
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			do init_type;
			location <- get_orthogonal_point(start_node, target_node, distance_to(start_node, target_node), rnd(dx, width_of_road_belong - dx));
			speed <- rnd(1.0, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
		
		create vehicle number: 1 {
			source_node <- road_network.vertices[1];
			destination_node <- road_network.vertices[0];
			type <- flip(0.3) ? 'CAR' : 'MOTORBIKE';
			do init_type;
			location <- get_orthogonal_point(start_node, target_node, distance_to(start_node, target_node), rnd(dx, width_of_road_belong - dx));
			speed <- rnd(1.0, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}
}

experiment straight_road {
	float minimum_cycle_duration <- MINIMUM_DURATION;
	output {
		display traffic_flow background:#grey {
			species road aspect:base;
			species roadNode aspect:base;
			species vehicle aspect:base;
		}
	}
}