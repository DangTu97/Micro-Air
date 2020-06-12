/***
* Name: case1
* Author: dang tu
* Description: one-way road
* Tags: Tag1, Tag2, TagN
***/

model case1
import "vehicle.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */

global {
	list<point> nodes <- [{10, 150}, {210, 150}];
	graph road_network <- graph([]);
	geometry shape <- square(environment_size);
	float step <- STEP;
	float road_width <- 5 #m;
	int unit <- 20;
	
	int nb_vehicle;
	float vehicle_average_speed <- 0.0;
	float vehicle_timer <- 0.0;
	int vehicle_counter <- 0;
	
	init {
//		write free_space;
		create road {
			shape <- polyline(nodes);
			geom_display <- shape + road_width;
			is_twoway <- false;
		}
		
		road_network <- as_edge_graph(road);
		
		//free space for motorbikes
		create free_myspace {
			geom <- polygon([nodes[0] + {0, -0.5*road_width}, nodes[0] + {0, road_width}, 
					nodes[0] + {2*road_width, road_width}, nodes[0] + {2*road_width, -0.5*road_width}]);
		}
		
		//free space for cars
		create free_myspace {
			geom <- polygon([nodes[0], nodes[0] + {road_width, 0}, nodes[0] + {road_width,-road_width}, nodes[0] + {0,-road_width}]);
		}
	}
	
	reflex init_traffic when: mod(cycle, 8) = 0 {	
		create vehicle number: 1 {
			int k <- rnd(100);
			if k <= 1 { name <- 'BUS';} 
			else if (k > 2) and (k <= 28) { name <- 'CAR';}
			else { name <- 'MOTORBIKE';}
			
			gender <- flip(0.5) ? 'MALE' : 'FEMALE';
			
			string maxspeed_string <- name + '_SAFESPEED_' + gender;
			
			if name = 'CAR' {
				length <- CAR_LENGTH;
				width <- CAR_WIDTH;
				df <- CAR_DF;
				db <- CAR_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
			} else if name = 'MOTORBIKE' {
				length <- MOTORBIKE_LENGTH;
				width <- MOTORBIKE_WIDTH;
				df <- MOTORBIKE_DF;
				db <- MOTORBIKE_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;	
			} else {
				length <- BUS_LENGTH;
				width <- BUS_WIDTH;
				df <- BUS_DF;
				db <- BUS_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				max_speed <- BUS_MAXSPEED;
			}
			
			speed <- INIT_SPEED;
			timer <- 0;
			
			switch maxspeed_string {
				match 'CAR_SAFESPEED_MALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? CAR_SAFESPEED_MALE : CAR_MAXSPEED;}
				match 'CAR_SAFESPEED_FEMALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? CAR_SAFESPEED_FEMALE : CAR_MAXSPEED;}
				match 'MOTORBIKE_SAFESPEED_MALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? MOTORBIKE_SAFESPEED_MALE : MOTORBIKE_MAXSPEED;}
				match 'MOTORBIKE_SAFESPEED_FEMALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? MOTORBIKE_SAFESPEED_FEMALE : MOTORBIKE_MAXSPEED;}
			}
			
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			start_node <- source_node;
			do compute_road_belong_nodes;
			target_node <- road_belong_nodes[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);

			// location
//			geometry free_space;
//			if name = 'CAR' {
//				free_space <- polygon([source_node, source_node + {road_width, 0}, source_node + {road_width,-road_width}, source_node + {0,-road_width}]);
//			} else {
////				free_space <- polygon([source_node, source_node + {0, road_width}, source_node + {road_width,road_width}, source_node + {road_width,0}]);
//				free_space <- polygon([source_node + {0, -0.5*road_width}, source_node + {0, road_width}, 
//					source_node + {2*road_width, road_width}, source_node + {2*road_width, -0.5*road_width}]);
//			}
			
			if name = 'MOTORBIKE' {
				location <- any_location_in(free_myspace(0).geom);
			} else {
				location <- any_location_in(free_myspace(1).geom);
			}
			do update_polygon;

			// compute future node to define turning line
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
					target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
				} else {
					if (alpha > 180) { alpha <- alpha - 180; if (alpha < 90) { alpha <- 180 - alpha; }}
					float k <- road_width/sin(180 - alpha);
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
			} else { target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90); }
		
//		if (cycle < 40) {max_speed <- max_speed / 2;}
		
		}
	}
	
	reflex update_info when:(mod(cycle,5*unit)=0) and (cycle != 0) {
		nb_vehicle <- length(vehicle);
//		list<float> all_speed <- [];
//		loop v over:vehicle {
//			float real_time <- v.timer/ (1/step);
//			if real_time > 0 {
//				float real_distance <- v.location.x - 10;
//				float average_speed_v <- real_distance/real_time;
//				all_speed <+ average_speed_v;
//			}
//		}
//		average_speed <- sum(all_speed)/length(all_speed);
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species vehicle aspect: base;
		}
		
		display my_chart refresh:every(200#cycle) {
			chart "Number of vehicles" position: {0, 0} size: {1.0,0.5} {
				data "vehicle" value:nb_vehicle color:#red;
			}
			chart "Average speed" position: {0, 0.5} size: {1.0,0.5}  {
				data "Speed" value:vehicle_average_speed color:#green;
			}
		}
	}
}
