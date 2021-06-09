/**
* Name: HKsimulation
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model HKsimulation
import '../species/vehicle.gaml'
import '../species/GIS_species.gaml'
import '../global_vars.gaml'
/* Insert your model definition here */

global {
	file road_node_file <- file('../includes/map/nodes.shp');
	file road_file <- file('../includes/map/edges.shp');
	file bound_file <- file('../includes/map/bound.shp');
	geometry shape <- envelope(bound_file);
	float step <- STEP;
	init {
//		create roadNode from:road_node_file with:[is_intersection:read("highway")='traffic_signals'] {
//			geom_display <- shape + 2;
//		}
		create roadNode from:road_node_file {
			geom_display <- shape + 2;
		}
		
		create road from:road_file with: [is_twoway::!bool(get('oneway')), junction::string(get('junction'))]{
			width <- 3.5#m;
			geom_display <- shape + width;
		}
		road_network <- directed(as_edge_graph(road));
		loop r over:road {
			loop rn over:roadNode {
				if rn.location = first(r.shape.points) or rn.location = last(r.shape.points) {
					r.endpoints <- r.endpoints + rn;
				}
			}
		}
		create vehicle number:1 {
			source_node <- one_of(road_network.vertices);
			destination_node <- one_of(road_network.vertices);
			type <- flip(0.0) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(dx, width_of_road_belong - dx));
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- true;
			do update_space;
		}
	}
	
	reflex init_traffic when:mod(cycle,1)=0 {
		create vehicle number:1 {
			source_node <- one_of(road_network.vertices);
			destination_node <- one_of(road_network.vertices);
			type <- flip(0.5) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(dx, width_of_road_belong - dx));
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}	
}

experiment HK_sim {
	float minimum_cycle_duration <- MINIMUM_DURATION;
	output {
		display my_display background:#grey {
			species road aspect:base;
//			species roadNode aspect: base;
			species vehicle aspect:base;
		}
	}
}