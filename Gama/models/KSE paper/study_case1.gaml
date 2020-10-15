/***
* Name: studycase
* Author: dang tu
* Description: traffic simulation for Giai Phong road at 7:30 am, the road is 200 meters long and 10.5 meters wide.
* Tags: Tag1, Tag2, TagN
***/

model studycase1
import "vehicle.gaml"
import "../global_variables.gaml"

/* Insert your model definition here */
global {
	list<point> nodes <- [{10, 150}, {210, 150}];
	graph road_network <- graph([]);
	geometry shape <- square(EVIRONMENT_SIZE);
	float step <- STEP;
	float road_width <- 5.25 #m;
	int unit <- 20;
	
	int nb_vehicle;
	float vehicle_average_speed <- 0.0;
	float vehicle_timer <- 0.0;
	int vehicle_counter <- 0;
	string model_name <- 'study_case';
	
	init {
		create road {
			shape <- polyline(nodes);
			road_width <- 5.25 #m;
			geom_display <- shape + road_width;
			is_twoway <- false;
		}
		
		road_network <- as_edge_graph(road);
		
		//for bus, car
		create init_space {
			location <- {10, 146.5};
			geom <- rectangle(5, 2);
		}
		
		//for car or motorbike
		create init_space {
			location <- {10, 148.5};
			geom <- rectangle(5, 2);
		}
		
		create init_space {
			location <- {10, 150};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 151};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 152};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 153};
			geom <- rectangle(1, 1);
		}
		create init_space {
			location <- {10, 154};
			geom <- rectangle(1, 1);
		}
	}
	
	//when:mod(cycle, 20)=0
	reflex init_traffic {	
		loop i from:0 to:6 {
			list<agent> var <- agents_overlapping(init_space(i).geom);
			if (length(var) <= 1 and i != 2) or (i = 2 and length(var) <= 2) {
				create vehicle number: 1 {
					if i = 0 { type <- flip(0.02) ? 'BUS' : 'CAR';}  
					else if i = 1 { type <- flip(0.5) ? 'CAR' : 'MOTORBIKE';}
					else {type <- 'MOTORBIKE';}
					
					timer <- 0;
					gender <- flip(0.5) ? 'MALE' : 'FEMALE';
					string my_string <- type + '_SAFESPEED_' + gender;
					switch my_string {
						match 'CAR_SAFESPEED_MALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? CAR_SAFESPEED_MALE : CAR_MAXSPEED;}
						match 'CAR_SAFESPEED_FEMALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? CAR_SAFESPEED_FEMALE : CAR_MAXSPEED;}
						match 'MOTORBIKE_SAFESPEED_MALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? MOTORBIKE_SAFESPEED_MALE : MOTORBIKE_MAXSPEED;}
						match 'MOTORBIKE_SAFESPEED_FEMALE' {max_speed <- flip(PROB_MAXSPEED_SAFESPEED) ? MOTORBIKE_SAFESPEED_FEMALE : MOTORBIKE_MAXSPEED;}
					}
					
					if type = 'CAR' {
						length <- CAR_LENGTH;
						width <- CAR_WIDTH;
						df <- CAR_DF;
						db <- CAR_DB;
						dx <- width/2 + db;
						dy <- length/2 + df;
					} else if type = 'MOTORBIKE' {
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
					width_size <- WIDTH_SIZE;
					minimun_length_size <- MINIMUM_LENGTH_SIZE;
					distance_check <- DISTANCE_CHECHK;
					acceleration_factor <- ACCELERATION_FACTOR;
					deceleration_factor <- DECELERATION_FACTOR;
					speed <- INIT_SPEED;
					prob_go_opposite <- PROB_GO_OPPOSITE;
					prob_turn_right <- PROB_TURN_RIGHT;
					
					display_polygon <- false;
					source_node <- road_network.vertices[0];
					final_node <- road_network.vertices[1];
					do compute_shortest_path;
					if length(shortest_path) = 0 { do die; }
					road_belong <-  shortest_path[0];
					start_node <- source_node;
					do compute_road_belong_nodes;
					target_node <- road_belong_nodes[1];
					angle <- angle_between(start_node, start_node + {10,0}, target_node);
					
					location <- init_space(i).location + {0.15 - rnd(0.3), 0.15 - rnd(0.3)};
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