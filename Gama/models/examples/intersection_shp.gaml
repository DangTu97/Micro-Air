/**
* Name: intersectionshp
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model intersectionshp
import '../species/vehicle.gaml'
import '../species/GIS_species.gaml'
import '../global_vars.gaml'

/* Insert your model definition here */
global {
	geometry shape <- square(200);
	graph road_network;
	float step <- STEP;
	file road_node_file <- file('../includes/1_intersection_nodes.shp');
	file road_file <- file('../includes/1_intersection_roads.shp');
	float road_width <- 8#m;
	init {
		map<point, list<road>> intersection_road_map <- [{100.0,100.0,0.0}::[road(1), road(2), road(0), road(3)]];
		create road from:road_file {
			width <- 8#m;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
	
		create roadNode from:road_node_file with:[is_intersection::(read('type')='intersection')]{
			geom_display <- shape + circle(0.3#m);
			if is_intersection {
				create intersection number:1 {
					location <- myself.location;
					center_location <- location;
					roads <- [road(1), road(2), road(0), road(3)];
					distance_from_center <- 8#m;
					n_strips <- [5, 10, 5];
					percents <- [1/4, 1/2, 1/4]; // sum all elements of percents should equal to 1;
					do build_attached_points;
					do build_geometry_matrix;
					myself.is_intersection <- true;
					myself.intersection_belong <- self;
					loop i from:0 to:length(attached_points) - 1 {
						int gap_time <- 10;
						create traffic_light {
							location <- myself.attached_points[i];
							shape <- circle(1);
							is_green <- false;
							t_g <- 100;
							t_y <- 15;
							t_r <- t_g + t_y + 2*gap_time;
							counter <- (i mod 2 = 0) ? 0 : t_g + t_y + gap_time;
							road_control <- myself.roads[i];
							myself.light_control <- myself.light_control + self;
						}
					}
					do build_conflict_list;
				}
			}
		}
		
		loop r over:road {
			loop rn over:roadNode {
				if rn.location = first(r.shape.points) or rn.location = last(r.shape.points) {
					r.endpoints <- r.endpoints + rn;
				}
			}
		}
		
		loop v over:road_network.vertices {
			list<road> my_roads <- road where (first(each.endpoints).location = v or last(each.endpoints).location = v);
			write "--";
			write v;
			write road_network neighbors_of v;
			write my_roads;
		}

		create vehicle number:1 {
			location <- {104, 115};
			source_node <- road_network.vertices[2];
			destination_node <- one_of(road_network.vertices[3], road_network.vertices[0], road_network.vertices[4]);
			type <- 'CAR';
			do init_type;
			speed <- 0.0;
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}
	
	reflex init_traffic when:mod(cycle, 10) = 0 {
		float p <- 0.6;
		float q <- 0.75;
		create vehicle number:1 {
			source_node <- road_network.vertices[2];
			destination_node <- flip(p) ? road_network.vertices[0] : one_of(road_network.vertices[3], road_network.vertices[4]);
			type <- flip(q) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
		
		create vehicle number:1 {
			source_node <- road_network.vertices[0];
			destination_node <- flip(p) ? road_network.vertices[2] : one_of(road_network.vertices[3], road_network.vertices[4]);
			type <- flip(q) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
		
		create vehicle number:1 {
			source_node <- road_network.vertices[3];
			destination_node <- flip(p) ? road_network.vertices[4] : one_of(road_network.vertices[2], road_network.vertices[0]);
			type <- flip(q) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
		
		create vehicle number:1 {
			source_node <- road_network.vertices[4];
			destination_node <- flip(p) ? road_network.vertices[3] : one_of(road_network.vertices[2], road_network.vertices[0]);
			type <- flip(q) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}
}

experiment intersection_shp {
	float minimum_cycle_duration <- MINIMUM_DURATION; 
	output {
		display traffic background:#grey {
			species road aspect:base;
			species roadNode aspect:base;
//			species intersection aspect:base;
			species traffic_light aspect:base;
			species vehicle aspect:base;
		}
	}
}
