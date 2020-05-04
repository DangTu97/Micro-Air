/***
* Name: realshpfile
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model realshpfile
import "../vehicle.gaml"

/* Insert your model definition here */

global {
	file shape_file_roads <- file("../includes/lines.shp");
	file shape_file_bound <- file("../includes/bound.shp");
	geometry shape <- envelope(shape_file_bound);
	init {
		create road from: shape_file_roads {
			geom_display <- shape + road_width;
			is_twoway <- true;
//			write shape;
		}
		road_network <- as_edge_graph(road);
		write road_network;
		
		create vehicle number: 100 {
			name <- flip(0.3) ? 'car' : 'motorbike';
			if name = 'car' {
				length <- 3.8 #m;
				width <- 1.5 #m;
				df <- 0.25 #m;
				db <- 0.15 #m;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- 0.1;
				max_speed <- rnd(0.4, 1.0) #m/#s;
			} else {
				length <- 1.8 #m;
				width <- 0.7 #m;
				df <- 0.15 #m;
				db <- 0.1 #m;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- 0.1;
				max_speed <- rnd(0.2, 0.7) #m/#s;
			}

			source_node <- one_of(road_network.vertices);
			final_node <-  one_of(road_network.vertices);
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			
			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob <- 0.0;
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
			
//			write future_node;
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
	
	reflex init_traffic when:cycle<0 {
		create vehicle number: 1 {
			name <- flip(0.3) ? 'car' : 'motorbike';
			if name = 'car' {
				length <- 3.8 #m;
				width <- 1.5 #m;
				df <- 0.25 #m;
				db <- 0.15 #m;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- 0.1;
				max_speed <- rnd(0.4, 1.0) #m/#s;
			} else {
				length <- 1.8 #m;
				width <- 0.7 #m;
				df <- 0.15 #m;
				db <- 0.1 #m;
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- 0.1;
				max_speed <- rnd(0.2, 0.7) #m/#s;
			}
			
			source_node <- road(13).shape.points[0];
			final_node <- road(7).shape.points[1];
			do compute_shortest_path;
			if length(shortest_path) = 0 { do die; }
			
			road_belong <-  shortest_path[0];
			display_polygon <- false;
			prob <- 0.0;
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
			
//			write future_node;
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

experiment my_experiment {
	float minimum_cycle_duration <- 0.01;
	output {
		display my_display background: #grey{
			species road aspect: base;
			species vehicle aspect: base;
		}
	}
}