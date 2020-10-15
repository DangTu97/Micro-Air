/***
* Name: toymodel
* Author: dang tu
* Description: simulation on two-way road
* Tags: Tag1, Tag2, TagN
***/

model toymodel

/* Insert your model definition here */
import "vehicle.gaml"
import "../global_variables.gaml"

global {
	list<point> nodes <- [{10, 50}, {210, 50}];
	graph road_network <- graph([]);
	geometry shape <- square(EVIRONMENT_SIZE);
	float step <- STEP;
	int unit <- 20;
	int traffic_volume_bottom <- 3; 
	int traffic_volume_top <- 5; 
	float PROB_GO_OPPOSITE <- 1.0;
	
	int nb_vehicle;
	float vehicle_average_speed <- 0.0;
	float vehicle_timer <- 0.0;
	int vehicle_counter <- 0;
	
	int nb_top; // number of vehicle at the top lane but move on bottom lane
	int nb_bottom; // number of vehicle at the bottom lane but move on top lane
	
	init {
		create road {
			shape <- polyline(nodes);
			road_width <- ROAD_WIDTH;
			geom_display <- shape + road_width;
			is_twoway <- true;
		}
		road_network <- as_edge_graph(road);
	}
	
	reflex init_traffic when: mod(cycle, unit) = 0 {
		create vehicle number: traffic_volume_bottom {
				int i <- rnd(1, 100);
				type <- (i < 2) ? 'BUS' : ( i < 20 ? 'CAR' : 'MOTORBIKE');
				
				if type = 'CAR' {
					length <- CAR_LENGTH;
					width <- CAR_WIDTH;
					df <- CAR_DF;
					db <- CAR_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- rnd(CAR_MAXSPEED/2, CAR_MAXSPEED);
				} else if type = 'MOTORBIKE' {
					length <- MOTORBIKE_LENGTH;
					width <- MOTORBIKE_WIDTH;
					df <- MOTORBIKE_DF;
					db <- MOTORBIKE_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- rnd(MOTORBIKE_MAXSPEED/2, MOTORBIKE_MAXSPEED);
				} else {
					length <- BUS_LENGTH;
					width <- BUS_WIDTH;
					df <- BUS_DF;
					db <- BUS_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- BUS_MAXSPEED;
				}
	
				width_size <- WIDTH_SIZE;
				minimun_length_size <- MINIMUM_LENGTH_SIZE;
				distance_check <- DISTANCE_CHECHK;
				acceleration_factor <- ACCELERATION_FACTOR;
				deceleration_factor <- DECELERATION_FACTOR;
				speed <- INIT_SPEED;
				prob_go_opposite <- PROB_GO_OPPOSITE;
				prob_turn_right <- PROB_TURN_RIGHT;
				
				display_polygon <- false;
				source_node <- nodes[0];
				final_node <-  nodes[1];
				on_right_side <- true;
				do compute_shortest_path;
				if length(shortest_path) = 0 { do die; }
				road_belong <-  shortest_path[0];
				start_node <- source_node;
				do compute_road_belong_nodes;
				target_node <- road_belong_nodes[1];
				angle <- angle_between(start_node, start_node + {10,0}, target_node);
				
				// location
				point p1 <- start_node;
				float a <- (target_node - start_node).x;
				float b <- (target_node - start_node).y;
				float d <- distance_to(target_node, start_node);
				point p1 <- start_node;
				point p2 <- p1 + {ROAD_WIDTH*a/d, ROAD_WIDTH*b/d};
				
				point center <- (p1 + p2)/2;
				float D <- 0.5*ROAD_WIDTH;
				point free_space_center <- center + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};
				if angle_between(start_node, target_node, free_space_center) > 180 {
					free_space_center <- center + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
				}	
				point p3 <- free_space_center*2 - p1;
				point p4 <- free_space_center*2 - p2;
				free_space <- polygon([p1,p2,p3,p4, p1]);
				location <- any_location_in(free_space);
				do update_polygon;
							
				point future_node;
				point x1;
				point x2;
				point x3;
				point x4; 
				// x1, x2, x3, x4 form a parallelogram
				int my_idx <- (road_belong_nodes index_of target_node);
				if (my_idx < length(road_belong_nodes) - 1) {
					future_node <- road_belong_nodes[my_idx + 1];
				} else {
					road my_feature_road <- get_next_road();
					if (my_feature_road != nil) {
						if (target_node = first(my_feature_road.shape.points)) {
							future_node <- my_feature_road.shape.points[1];
						} else if (target_node = last(my_feature_road.shape.points)) {
							future_node <- reverse(my_feature_road.shape.points)[1];
						}
					}
				}
				
				if (future_node != nil)  {
					float alpha <- angle_between(target_node, start_node, future_node);
					float k;
					if (alpha = 180) {
						target_space <- polyline([target_node - {1.5*ROAD_WIDTH, 0}, target_node + {1.5*ROAD_WIDTH, 0}]) rotated_by (angle + 90);
					} else {
						if (alpha > 180) { alpha <- alpha - 180; if (alpha < 90) { alpha <- 180 - alpha; }}
						float k <- ROAD_WIDTH/sin(180 - alpha);
						x2 <- target_node;
						float a <- (start_node - target_node).x;
						float b <- (start_node - target_node).y;
						float d <- distance_to(target_node, start_node);
						point x1 <- x2 + {k*a/d, k*b/d};
						
						float A <- (future_node - target_node).x;
						float B <- (future_node - target_node).y;
						float D <- distance_to(future_node, target_node);
						point x3 <- x2 + {k*A/D, k*B/D};
						
						point center <- (x1 + x3)/2;
						point x4 <- center*2 - x2;
						point x4 <- center*2 - x2;
						point newpoint <- x2*2 - x4;
						target_space <- polyline([newpoint,x4]);
					}
				} else { target_space <- polyline([target_node - {1.5*ROAD_WIDTH, 0}, target_node + {1.5*ROAD_WIDTH, 0}]) rotated_by (angle + 90); }
			}
			
			create vehicle number: traffic_volume_top {
				int i <- rnd(1, 100);
				type <- (i < 2) ? 'BUS' : ( i < 20 ? 'CAR' : 'MOTORBIKE');
				
				if type = 'CAR' {
					length <- CAR_LENGTH;
					width <- CAR_WIDTH;
					df <- CAR_DF;
					db <- CAR_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- rnd(CAR_MAXSPEED/2, CAR_MAXSPEED);
				} else if type = 'MOTORBIKE' {
					length <- MOTORBIKE_LENGTH;
					width <- MOTORBIKE_WIDTH;
					df <- MOTORBIKE_DF;
					db <- MOTORBIKE_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- rnd(MOTORBIKE_MAXSPEED/2, MOTORBIKE_MAXSPEED);
				} else {
					length <- BUS_LENGTH;
					width <- BUS_WIDTH;
					df <- BUS_DF;
					db <- BUS_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- BUS_MAXSPEED;
				}
	
				width_size <- WIDTH_SIZE;
				minimun_length_size <- MINIMUM_LENGTH_SIZE;
				distance_check <- DISTANCE_CHECHK;
				acceleration_factor <- ACCELERATION_FACTOR;
				deceleration_factor <- DECELERATION_FACTOR;
				speed <- INIT_SPEED;
				prob_go_opposite <- PROB_GO_OPPOSITE;
				prob_turn_right <- PROB_TURN_RIGHT;
				
				display_polygon <- false;
				source_node <- nodes[1];
				final_node <-  nodes[0];
				on_right_side <- true;
				do compute_shortest_path;
				if length(shortest_path) = 0 { do die; }
				road_belong <-  shortest_path[0];
				start_node <- source_node;
				do compute_road_belong_nodes;
				target_node <- road_belong_nodes[1];
				angle <- angle_between(start_node, start_node + {10,0}, target_node);
				
				// location
				point p1 <- start_node;
				float a <- (target_node - start_node).x;
				float b <- (target_node - start_node).y;
				float d <- distance_to(target_node, start_node);
				point p1 <- start_node;
				point p2 <- p1 + {ROAD_WIDTH*a/d, ROAD_WIDTH*b/d};
				
				point center <- (p1 + p2)/2;
				float D <- 0.5*ROAD_WIDTH;
				point free_space_center <- center + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};
				if angle_between(start_node, target_node, free_space_center) > 180 {
					free_space_center <- center + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
				}	
				point p3 <- free_space_center*2 - p1;
				point p4 <- free_space_center*2 - p2;
				free_space <- polygon([p1,p2,p3,p4, p1]);
				location <- any_location_in(free_space);
				do update_polygon;
							
				point future_node;
				point x1;
				point x2;
				point x3;
				point x4; 
				// x1, x2, x3, x4 form a parallelogram
				int my_idx <- (road_belong_nodes index_of target_node);
				if (my_idx < length(road_belong_nodes) - 1) {
					future_node <- road_belong_nodes[my_idx + 1];
				} else {
					road my_feature_road <- get_next_road();
					if (my_feature_road != nil) {
						if (target_node = first(my_feature_road.shape.points)) {
							future_node <- my_feature_road.shape.points[1];
						} else if (target_node = last(my_feature_road.shape.points)) {
							future_node <- reverse(my_feature_road.shape.points)[1];
						}
					}
				}
				
				if (future_node != nil)  {
					float alpha <- angle_between(target_node, start_node, future_node);
					float k;
					if (alpha = 180) {
						target_space <- polyline([target_node - {1.5*ROAD_WIDTH, 0}, target_node + {1.5*ROAD_WIDTH, 0}]) rotated_by (angle + 90);
					} else {
						if (alpha > 180) { alpha <- alpha - 180; if (alpha < 90) { alpha <- 180 - alpha; }}
						float k <- ROAD_WIDTH/sin(180 - alpha);
						x2 <- target_node;
						float a <- (start_node - target_node).x;
						float b <- (start_node - target_node).y;
						float d <- distance_to(target_node, start_node);
						point x1 <- x2 + {k*a/d, k*b/d};
						
						float A <- (future_node - target_node).x;
						float B <- (future_node - target_node).y;
						float D <- distance_to(future_node, target_node);
						point x3 <- x2 + {k*A/D, k*B/D};
						
						point center <- (x1 + x3)/2;
						point x4 <- center*2 - x2;
						point x4 <- center*2 - x2;
						point newpoint <- x2*2 - x4;
						target_space <- polyline([newpoint,x4]);
					}
				} else { target_space <- polyline([target_node - {1.5*ROAD_WIDTH, 0}, target_node + {1.5*ROAD_WIDTH, 0}]) rotated_by (angle + 90); }
			}
	}
	
	reflex count_vehicles {
		nb_top <- length(vehicle where (each.source_node = nodes[1] and each.on_right_side = false));
		nb_bottom <- length(vehicle where (each.source_node = nodes[0] and each.on_right_side = false));
		if (nb_bottom + nb_top >= 4) and (abs(nb_bottom - nb_top) >= 3) and cycle > 500  and min(nb_bottom, nb_top) > 0{
			do pause;
		}
	}
}

species my_species {
	aspect default {
		draw shape color:#red;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species vehicle aspect: base;
		}
		monitor "Upper lane flow" value: 5;
		monitor "Lower lane flow" value: 1;
		monitor "Number of wrong vehicles on upper lane" value: nb_top;
		monitor "Number of wrong vehicles on lower lane" value: nb_bottom;
	}
}