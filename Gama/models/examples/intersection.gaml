/***
* Name: intersection
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model intersection
import "../vehicle.gaml"
import "../global_variables.gaml"
/* Insert your model definition here */

global {
	geometry shape <- square(500);
	float step <- 0.05#s;
	int nb_vehicles;
	int traffic_volume <- 5;
	int road_width <- ROAD_WIDTH;
	init {
		list<list<point>> my_nodes <- [[{100,50}, {100,100}], [{100,100}, {100,150}], [{50,100}, {100,100}], [{100,100}, {150,100}]];
		loop nodes over:my_nodes {
			create road {
				shape <- polyline(nodes);
				road_width <- ROAD_WIDTH;
				geom_display <- (shape + road_width);
				is_twoway <- true;
			}
		}
		
		create traffic_light {
			location <- {100, 99 - road_width};
			shape <- circle(1);
			counter <- 0;
			is_green <- false;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			direction_control <- 90;
			my_geom <- polyline([location, location + {-road_width, 0}]);
		}
		
		create traffic_light {
			location <- {100, 101 + road_width};
			shape <- circle(1);
			counter <- 0;
			is_green <- false;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			direction_control <- 270;
			my_geom <- polyline([location, location + {road_width, 0}]);
		}
		
		create traffic_light {
			location <- {99 - road_width, 100};
			shape <- circle(1);
			counter <- RED_TIME;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			is_green <- true;
			direction_control <- 0;
			my_geom <- polyline([location, location + {0, road_width}]);
		}
		
		create traffic_light {
			location <- {101 + road_width, 100};
			shape <- circle(1);
			counter <- RED_TIME;
			t_g <- GREEN_TIME;
			t_y <- YELLOW_TIME;
			t_r <- RED_TIME;
			is_green <- true;
			direction_control <- 180;
			my_geom <- polyline([location, location + {0, -road_width}]);
		}
		
		loop i from: 0 to:3 {
			road(i).light_belong <- traffic_light(i);
		}
		
		road_network <- as_edge_graph(road);
		write road_network;
		
		create block_space {
			location <- {100, 100};
			shape <- circle(2);
		}
		
		// ----------------------
		create init_space {
			location <- {100,50} + {-road_width/2, 2};
			my_point <- {100,50};
			geom <- rectangle(road_width, 2);
		}
		
		create init_space {
			location <- {100,150} + {road_width/2, -2};
			my_point <- {100,150};
			geom <- rectangle(road_width, 2);
		}
		
		create init_space {
			location <- {50,100} + {2,road_width/2};
			my_point <- {50,100};
			geom <- rectangle(road_width, 2) rotated_by 90;
		}
		
		create init_space {
			location <- {150,100} + {-2,-road_width/2};
			my_point <- {150,100};
			geom <- rectangle(road_width, 2) rotated_by 90;
		}
	}
	
	reflex init_traffic when:mod(cycle,20)=0 {
		list<point> targets <- [];
		loop i from:0 to:3 {
			if length(agents_overlapping(init_space(i).geom)) = 2 {
				targets <+ init_space(i).my_point;
			}
		}
//		write length(targets);
		
		if length(targets) > 1 {
			create vehicle number: traffic_volume {
//				type <- flip(CAR_PERCENT) ? 'CAR' : 'MOTORBIKE';
				
				int i <- rnd(1, 100);
				type <- (i < 2) ? 'BUS' : ( i < 20 ? 'CAR' : 'MOTORBIKE');
					
				if type = 'CAR' {
					length <- CAR_LENGTH;
					width <- CAR_WIDTH;
					df <- CAR_DF;
					db <- CAR_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- CAR_MAXSPEED;
				} else if type = 'MOTORBIKE' {
					length <- MOTORBIKE_LENGTH;
					width <- MOTORBIKE_WIDTH;
					df <- MOTORBIKE_DF;
					db <- MOTORBIKE_DB;
					dx <- width/2 + db;
					dy <- length/2 + df;
					max_speed <- MOTORBIKE_MAXSPEED;
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
				source_node <- one_of(targets);
				final_node <-  one_of(targets where (each != source_node));
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
				point p2 <- p1 + {road_width*a/d, road_width*b/d};
				
				point center <- (p1 + p2)/2;
				float D <- 0.5*road_width;
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
				
	//			write future_node;
				if (future_node != nil)  {
					float alpha <- angle_between(target_node, start_node, future_node);
					float k;
	//				write alpha;
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
		
		nb_vehicles <- length(vehicle);
	}
}



experiment my_experiment {
	float minimum_cycle_duration <- TIME_STEP;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species traffic_light aspect: base;
//			species free_myspace aspect: base;
//			species block_space aspect: base;
			species vehicle aspect: base;
		}
		
		display my_chart refresh:every(200#cycle) {
			chart "Number of vehicles" position: {0, 0.5} size: {1.0,0.5}  {
				data "Vehicle" value:nb_vehicles color:#green;
			}
		}
		monitor "No. vehicles" value: nb_vehicles;
		//monitor "Traffic volumne" value: traffic_volume;
	}
}