/***
* Name: case1
* Author: dang tu
* Description: one-way road
* Tags: Tag1, Tag2, TagN
***/

model case12
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
	float average_speed <- 0.0;
	float vehicle_timer <- 0.0;
	int vehicle_counter <- 0;
	
	init {
		create traffic_light {
			location <- {210, 150};
			is_traffic_signal <- true;
			shape <- circle(0.5);
			counter <- 0;
			is_green <- false;
			direction_control <- 0;
			my_geom <- polyline([location + {0,-road_width}, location + {0,road_width}]);
		}
		
		create road {
			shape <- polyline(nodes);
			geom_display <- shape + road_width;
			is_twoway <- false;
//			road(0).light_belong <- traffic_light(0);
		}
		
		road_network <- as_edge_graph(road);
		
		create free_myspace {
			location <- {10, 146};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 147};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 148};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 149};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 150};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 151};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 152};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 153};
			geom <- rectangle(2, 1);
		}
		create free_myspace {
			location <- {10, 154};
			geom <- rectangle(2, 1);
		}
		
	}
	
	//when:mod(cycle, 20)=0
	reflex init_traffic when:mod(cycle, 1) = 0 {	
		loop i from:0 to:8 {
			list<agent> var <- agents_overlapping(free_myspace(i).geom);
			if (length(var) <= 1 and i != 4) or (i = 4 and length(var) <= 2) {
//			if true {
				create vehicle number: 1 {
					name <- 'MOTORBIKE';
					gender <- flip(0.5) ? 'MALE' : 'FEMALE';
					
					//age
					int k <- rnd(100);
					if (k <= 50) {age <- 'YOUNG';}
					else if (k > 50 and k <= 93) {age <- 'MIDDLEAGED';}
					else {age <- 'OLD';}
					
					string maxspeed_string <- name + '_MAXSPEED_' + age + '_' + gender;
					string safespeed_string <- name + '_SAFESPEED_' + age + '_' + gender;
					
					if name = 'CAR' {
						length <- CAR_LENGTH;
						width <- CAR_WIDTH;
						df <- CAR_DF;
						db <- CAR_DB;
						dx <- width/2 + db;
						dy <- length/2 + df;
					} else {
						length <- MOTORBIKE_LENGTH;
						width <- MOTORBIKE_WIDTH;
						df <- MOTORBIKE_DF;
						db <- MOTORBIKE_DB;
						dx <- width/2 + db;
						dy <- length/2 + df;	
					}
					
					switch maxspeed_string {
						match 'MOTORBIKE_MAXSPEED_YOUNG_MALE' {max_speed <- flip(YOUNGLE_MALE_PROB) ? MOTORBIKE_MAXSPEED_YOUNG_MALE : MOTORBIKE_SAFESPEED_YOUNG_MALE;}
						match 'MOTORBIKE_MAXSPEED_MIDDLEAGED_MALE' {max_speed <- flip(MIDDLEAGED_MALE_PROB) ? MOTORBIKE_MAXSPEED_MIDDLEAGED_MALE : MOTORBIKE_SAFESPEED_MIDDLEAGED_MALE;}
						match 'MOTORBIKE_MAXSPEED_OLD_MALE' {max_speed <- flip(OLD_MALE_PROB) ? MOTORBIKE_MAXSPEED_OLD_MALE : MOTORBIKE_SAFESPEED_OLD_MALE;}
						match 'MOTORBIKE_MAXSPEED_YOUNG_FEMALE' {max_speed <- flip(YOUNGLE_FEMALE_PROB) ? MOTORBIKE_MAXSPEED_YOUNG_FEMALE : MOTORBIKE_SAFESPEED_YOUNG_FEMALE;}
						match 'MOTORBIKE_MAXSPEED_MIDDLEAGED_FEMALE' {max_speed <- flip(MIDDLEAGED_FEMALE_PROB) ? MOTORBIKE_MAXSPEED_MIDDLEAGED_FEMALE : MOTORBIKE_SAFESPEED_MIDDLEAGED_FEMALE;}
						match 'MOTORBIKE_MAXSPEED_OLD_FEMALE' {max_speed <- flip(OLD_FEMALE_PROB) ? MOTORBIKE_MAXSPEED_OLD_FEMALE : MOTORBIKE_SAFESPEED_OLD_FEMALE;}
						
						match 'CAR_MAXSPEED_YOUNG_MALE' {max_speed <- flip(YOUNGLE_MALE_PROB) ? CAR_MAXSPEED_YOUNG_MALE : CAR_SAFESPEED_YOUNG_MALE;}
						match 'CAR_MAXSPEED_MIDDLEAGED_MALE' {max_speed <- flip(MIDDLEAGED_MALE_PROB) ? CAR_MAXSPEED_MIDDLEAGED_MALE : CAR_SAFESPEED_MIDDLEAGED_MALE;}
						match 'CAR_MAXSPEED_OLD_MALE' {max_speed <- flip(OLD_MALE_PROB) ? CAR_MAXSPEED_OLD_MALE : CAR_SAFESPEED_OLD_MALE;}
						match 'CAR_MAXSPEED_YOUNG_FEMALE' {max_speed <- flip(YOUNGLE_FEMALE_PROB) ? CAR_MAXSPEED_YOUNG_FEMALE : CAR_SAFESPEED_YOUNG_FEMALE;}
						match 'CAR_MAXSPEED_MIDDLEAGED_FEMALE' {max_speed <- flip(MIDDLEAGED_FEMALE_PROB) ? CAR_MAXSPEED_MIDDLEAGED_FEMALE : CAR_SAFESPEED_MIDDLEAGED_FEMALE;}
						match 'CAR_MAXSPEED_OLD_FEMALE' {max_speed <- flip(OLD_FEMALE_PROB) ? CAR_MAXSPEED_OLD_FEMALE : CAR_SAFESPEED_OLD_FEMALE;}
					}
					
					speed <- INIT_SPEED;
					max_speed <- rnd(8.0, 14.0) #m/#s;
					timer <- 0;
					
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
					
					location <- free_myspace(i).location + {0.15 - rnd(0.3), 0.15 - rnd(0.3)};
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
				}
			}
		}
	}
	
	reflex update_info when:(mod(cycle,5*unit)=0) and (cycle != 0) {
		nb_vehicle <- length(vehicle);
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species free_myspace aspect: base;
			species traffic_light aspect: base;
			species vehicle aspect: base;
		}
		
		display my_chart refresh:every(200#cycle) {
			chart "Number of vehicles" position: {0, 0} size: {1.0,0.5} {
				data "vehicle" value:nb_vehicle color:#red;
			}
			chart "Average speed" position: {0, 0.5} size: {1.0,0.5}  {
				data "Speed" value:vehicle_timer color:#green;
			}
		}
	}
}
