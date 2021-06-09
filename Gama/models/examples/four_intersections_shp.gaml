/**
* Name: fourintersectionsshp
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model fourintersectionsshp
import '../species/vehicle.gaml'
import '../species/GIS_species.gaml'
import '../global_vars.gaml'
/* Insert your model definition here */

global {
	geometry shape <- square(350);
	graph road_network;
	float step <- STEP;
	file road_node_file <- file('../includes/4_intersection_nodes.shp');
	file road_file <- file('../includes/4_intersection_roads.shp');
	float road_width <- 8#m;
	init {
		create road from:road_file {
			width <- 8#m;
			geom_display <- shape + width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
		map<int, list<road>> test_intersection_road_map <- [0::[road(1), road(2), road(0), road(3)],
														 	1::[road(6), road(3), road(4), road(5)],
															2::[road(8), road(7), road(1), road(9)],
														 	3::[road(11), road(9), road(6), road(10)]];
		int count <- 0;
		create roadNode from:road_node_file with:[is_intersection::(read('type')='intersection')]{
			geom_display <- shape + circle(0.3#m);
			if is_intersection {
				create intersection number:1 {
					location <- myself.location;
					center_location <- location;
					roads <- test_intersection_road_map[count];
					count <- count + 1;
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
		
//		loop rn over:roadNode {
//			write rn.location;
//		}

//		loop rn over:road_network.vertices {
//			write rn;
//		}

		create vehicle number:1 {
			source_node <- one_of([roadNode(5), roadNode(7), roadNode(6), roadNode(1), roadNode(0), roadNode(2), roadNode(3), roadNode(4)]) as point;
			destination_node <- one_of([roadNode(5), roadNode(7), roadNode(6), roadNode(1), roadNode(0), roadNode(2), roadNode(3), roadNode(4)] - source_node) as point;
			type <- flip(0.5) ? 'MOTORBIKE' : 'CAR';
			do init_type;
			location <- get_orthogonal_point(source_node, target_node, distance_to(source_node, target_node), rnd(0.2, 0.8)*width_of_road_belong);
			speed <- rnd(max_speed/2, max_speed);
			status <- 'inside_road';
			debug <- false;
			do update_space;
		}
	}
	
//	when:mod(cycle,2)=0
	reflex init_traffic {
		float q <- 0.75;
		create vehicle number:1 {
			source_node <- one_of([roadNode(5), roadNode(7), roadNode(6), roadNode(1), roadNode(0), roadNode(2), roadNode(3), roadNode(4)]) as point;
			destination_node <- one_of([roadNode(5), roadNode(7), roadNode(6), roadNode(1), roadNode(0), roadNode(2), roadNode(3), roadNode(4)] - source_node) as point;
			if source_node = destination_node { do die; }
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

experiment multi_intersections {
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