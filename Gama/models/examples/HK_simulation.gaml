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
		
		loop v over:road_network.vertices {
			list<point> neighbors <- road_network neighbors_of v;
			if length(neighbors) = 1 {
				write "--";
				write v;
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
	
	reflex init_traffic when:mod(cycle,2)=0 {
		create vehicle number:1 {
			source_node <- one_of([roadNode(9), roadNode(10), roadNode(47), roadNode(23)]).location;
			destination_node <- one_of([roadNode(53), roadNode(42), roadNode(20), roadNode(27), roadNode(55), roadNode(14), roadNode(34), roadNode(50),
							            roadNode(11), roadNode(7)]).location;
			type <- flip(0.5) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- flip(0.5) ? get_orthogonal_point(start_node, target_node, distance_to(start_node, target_node), rnd(dx, width_of_road_belong - dx)) :
									start_node*2 - get_orthogonal_point(start_node, target_node, distance_to(start_node, target_node), rnd(dx, width_of_road_belong - dx));
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
		
		create vehicle number:1 {
			source_node <- one_of([roadNode(53), roadNode(42), roadNode(20), roadNode(27), roadNode(55), roadNode(14), roadNode(34), roadNode(50)]).location;
			destination_node <- one_of([roadNode(53), roadNode(42), roadNode(20), roadNode(27), roadNode(55), roadNode(14), roadNode(34), roadNode(50),
							            roadNode(11), roadNode(7)]).location;
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