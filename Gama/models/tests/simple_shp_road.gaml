/***
* Name: singleroad
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model singleroad
import "../vehicle.gaml"
import "../global_variables.gaml"


/* Insert your model definition here */

global {
	file shape_file_roads <-  file("../includes/multilines.shp");
	geometry shape <- envelope(shape_file_roads) + 2*road_width;
	bool  display3D<- false;
	init {
		create road from: shape_file_roads {
			geom_display <- shape + road_width;
			is_twoway <- true;
//			write length(shape.points);
		}
		
		list<point> nodes <- road(0).shape.points;
		loop node over:nodes {
			create my_species {
				location <- node;
				shape <- circle(road_width);
			}
		}
		
		road_network <- as_edge_graph(road);
		write road_network;
	}
	
	reflex init_traffic when:mod(cycle,200) = 0{
		create vehicle number: 20 {
			name <- flip(0.3) ? 'car' : 'motorbike';
			if name = 'car' {
				length <- CAR_LENGTH;
				width <- CAR_WIDTH;
				df <- CAR_DF;
				db <- CAR_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- INIT_SPEED;
				max_speed <- 0.2 + rnd(CAR_MAXSPEED - 0.2);
			} else {
				length <- MOTORBIKE_LENGTH;
				width <- MOTORBIKE_WIDTH;
				df <- MOTORBIKE_DF;
				db <- MOTORBIKE_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- INIT_SPEED;
				max_speed <- 0.2 + rnd(MOTORBIKE_MAXSPEED - 0.2);
			}
			
			source_node <- road_network.vertices[1];
			final_node <- road_network.vertices[0];
			do compute_shortest_path;
//			write shortest_path;

			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			location <- source_node + {rnd(3.0), rnd(3.0)};
			start_node <- road_belong.shape.points[5];
			target_node <- road_belong.shape.points[4];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			do compute_road_belong_nodes;
			do update_polygon;
			
//			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
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
					} else if (start_node = last(my_feature_road.shape.points)) {
						future_node <- reverse(my_feature_road.shape.points)[1];
					}
				}
			}
			
			if (future_node != nil) {
				float alpha <- angle_between(target_node, start_node, future_node);
				if (alpha > 180) { alpha <- alpha - 180; if (alpha < 90) { alpha <- 180 - alpha; }}
//				write alpha;
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
			} else { target_space <- target_node; }
		}
		
		create vehicle number: 15 {
			name <- flip(0.3) ? 'car' : 'motorbike';
			if name = 'car' {
				length <- CAR_LENGTH;
				width <- CAR_WIDTH;
				df <- CAR_DF;
				db <- CAR_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- INIT_SPEED;
				max_speed <- 0.2 + rnd(CAR_MAXSPEED - 0.2);
			} else {
				length <- MOTORBIKE_LENGTH;
				width <- MOTORBIKE_WIDTH;
				df <- MOTORBIKE_DF;
				db <- MOTORBIKE_DB;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- INIT_SPEED;
				max_speed <- 0.2 + rnd(MOTORBIKE_MAXSPEED - 0.2);
			}
			
			source_node <- road_network.vertices[0];
			final_node <- road_network.vertices[1];
			do compute_shortest_path;
//			write shortest_path;
	
			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob_go_opposite <- PROB_GO_OPPOSITE;
			prob_turn_right <- PROB_TURN_RIGHT;
			location <- source_node + {rnd(1.0), rnd(3.0)};
			start_node <- road_belong.shape.points[0];
			target_node <- road_belong.shape.points[1];
			angle <- angle_between(start_node, start_node + {10,0}, target_node);
			do compute_road_belong_nodes;
			do update_polygon;
			
//			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
			
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
					} else if (start_node = last(my_feature_road.shape.points)) {
						future_node <- reverse(my_feature_road.shape.points)[1];
					}
				}
			}
			
			if (future_node != nil) {
				float alpha <- angle_between(target_node, start_node, future_node);
				if (alpha > 180) { alpha <- alpha - 180; if (alpha < 90) { alpha <- 180 - alpha; }}
//				write alpha;
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
			} else { target_space <- target_node; }
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
//			species my_species;
			species vehicle aspect: base;
		}
	}
}