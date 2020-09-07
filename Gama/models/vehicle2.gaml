/***
* Name: vehicle2
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model vehicle2
import "global_variables.gaml"
/* Insert your model definition here */

global {
	graph road_network;
}

species traffic_light {
	int counter;
	rgb color;
	float t_g;
	float t_y;
	float t_r;
	
	// additional variables
	bool is_green;
	geometry my_geom;
	float direction_control;
	
	reflex change_color {
		counter <- counter + 1;
        if (counter >= t_g + t_y + t_r) { 
            counter <- 0;
        } else if (counter <= t_g ) {
        	color <- #green;
        	is_green <- true;
        } else if ((counter > t_g) and (counter <= t_g + t_y)) {
        	color <- #yellow;
        	is_green <- false;
        } else {
        	color <- #red;
        	is_green <- false;
        }
	}
	
	aspect base {
		draw shape at:location color:color;
		draw my_geom color:#blue;
	}
}

species road { 
	bool is_twoway;
	float width;
	geometry geom_display;
	rgb color <- #white;
	traffic_light light_belong;
	
	aspect base {    
		draw geom_display color: color border:#white ;
		if is_twoway {
			draw shape color:#grey;
		}
	}  
}

species vehicle skills:[moving] {
	string type;
	float length;
	float width;
	float df;
	float db;
	float dx; // (db + width/2)
	float dy; // (df + lenght/2)
	float max_speed;
	point source_node;
	point final_node;
	point target;
	point start_node;
	point target_node;
	point future_node;
	float angle;
	list<road> shortest_path;
	geometry current;
	geometry front;
	geometry left;
	geometry right;
	bool check_go_straight;
	bool check_turn_left;
	bool check_turn_right;
	float acceleration_factor;
	float deceleration_factor;
	
	// additional attributes
	road road_belong;
	list<point> road_belong_nodes; //with direction
	list<vehicle> vehicle_front;
	list<vehicle> vehicle_left;
	list<vehicle> vehicle_right;
	float distance_check;
	float minimun_length_size;
	bool display_polygon;
	bool on_right_side;
	float polygon_width_size; // for right and left polygons
	float prob_go_opposite; // probability to go opposite road
	float prob_turn_right; // probability to turn right
	
	geometry transfer_geom;
	bool is_transferred;
	
	bool check_is_on_road(geometry geom) {
		bool val <- false;
		if (geom.location overlaps road_belong.geom_display) {
			val <- true;
		} 
		
		return val;
	}
	
	bool check_on_right_side(geometry geom) {
		bool val <- false;
		if (angle_between(start_node, target_node, geom.location) < 180) {
			val <- true;
		} 
		
		return val;
	}
	
	action update_angle {
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
	}
	
	action update_polygon {
		// polygons for vehicle
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
						  location + {- dy, dx}, location + {-dy, -dx}]) rotated_by heading;
	    
	    
	    float d <- distance_to(start_node, target_node);
		float a <- (target_node - start_node).location.x;
		float b <- (target_node - start_node).location.y;    	
		point front_point;		  
		float size <- (speed/max_speed);
	    //ddy: dynamic length dy of front polygon: length = [minimun_length_size + speed/max_speed] * length of vehicle
		float ddy <- minimun_length_size*dy + size*dy;
		
		float h <- (1.0 + minimun_length_size + size)*dy;
		float xx <- h*cos(angle);
		float yy <- h*sin(angle);
		front_point <- location + {xx, yy};
		front <- polygon([front_point + {ddy, -dx}, front_point + {ddy, dx},
						  front_point + {- ddy, dx}, front_point + {-ddy, -dx}]) rotated_by angle;			  
		
		float D <- 1.5*dx;
		// left location with left polygon of size (dx,dy)
		point left_point <- front_point + {b*D/sqrt(a*a + b*b), - a*D/sqrt(a*a + b*b)};
		
		left <- polygon([left_point + {ddy, -polygon_width_size*dx}, left_point + {ddy, polygon_width_size*dx},
						  left_point + {- ddy, polygon_width_size*dx}, left_point + {-ddy, -polygon_width_size*dx}]) rotated_by angle;
						  
		// right location with right polygon of size (dx,dy)
		point right_point <- front_point + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
			  
		right <- polygon([right_point + {ddy, -polygon_width_size*dx}, right_point + {ddy, polygon_width_size*dx},
						  right_point + {- ddy, polygon_width_size*dx}, right_point + {-ddy, -polygon_width_size*dx}]) rotated_by angle;				
	}
	
	action compute_shortest_path {
   		path path_computed <- path_between(road_network, source_node, final_node);
   		list road_id_list <- string(path_computed) split_with('[()],road as path');
		if (length(road_id_list) > 0) {
			loop i from: 0 to:length(road_id_list) - 1 {
				shortest_path <+ road(int(road_id_list[i]));
			}
		}  
	}
	
	action accelerate {
		speed <- min(max_speed, speed + acceleration_factor);
	}
	
	action decelerate {
		speed <- max(0.0, speed - deceleration_factor);
	}
	
	action compute_road_belong_nodes {
		if (start_node = first(road_belong.shape.points)) {
			road_belong_nodes <- road_belong.shape.points;
		} else if (start_node = last(road_belong.shape.points)) {
			road_belong_nodes <- reverse(road_belong.shape.points);
		}
	}
	
	road get_next_road {
		road next_road;
		int index <- (shortest_path index_of road_belong);
		return (index < length(shortest_path) - 1) ? shortest_path[index + 1] : nil;
	}
	
	action change_segment {
		// change start node, target node, road belong, angle
		start_node <- target_node;
		int idx <- (road_belong_nodes index_of target_node);
		if (idx < length(road_belong_nodes) - 1) {
			target_node <- road_belong_nodes[idx + 1];
		} else {
			road_belong <- get_next_road();
			if (road_belong = nil) {
				do die; // or do something
			} else {
				do compute_road_belong_nodes;
				target_node <- road_belong_nodes[1];
			}
		}
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
	}
	
	action control_twoway {
		on_right_side <- check_on_right_side(current);
		if (on_right_side = false) { // if on left side
			//ver1
			if (check_turn_right = true) {
				do accelerate;
				target <- right.location;
			} else if (check_go_straight = true) {
				do accelerate;
				target <- front.location;
			} else if ((check_turn_left = true) and (first(vehicle_front) != nil) 
			and first(vehicle_front).target_node = target_node) {
				if flip(prob_go_opposite) {
					do accelerate;
					target <- left.location;
				}
			} else {
				target <- right.location;
				speed <- max(speed - deceleration_factor, 0.02);
			}

			//ver2
			write "false";
//			target <- right.location;
//			if (check_turn_right = true) {
//				do accelerate;
//			} else {
//				speed <- 0;
//			}
		}
		
		else { // if on right side
			if (check_go_straight = true) {
				do accelerate;
				target <- front.location;
			} else if (check_turn_left = true) and (check_on_right_side(left) = true) {
				do accelerate;
				target <- left.location;
			} else if (check_turn_right = true) and flip(prob_turn_right) {
				do accelerate;
				target <- right.location;
			} else if (check_turn_left = true) and (check_on_right_side(left) = false ) and flip(prob_go_opposite) {
				do accelerate;
				target <- left.location;
			} else {
				target <- front.location;
				do decelerate();
			}
		}
	}
	
	action control_oneway {
		if (check_go_straight = true) {
			do accelerate;
			target <- front.location;
		} else if (check_turn_left = true) {
			// turn teft
			do accelerate;
			target <- left.location;
		} else if (check_turn_right = true) and flip(prob_turn_right) {
			do accelerate;
			target <- right.location;
		} else {
			target <- front.location;
			do decelerate();
		}
	}
	
	action define_new_target {
		if road_belong.is_twoway = true {
			if (front overlaps transfer_geom) {
				target <- front.location;
			} else {
				do control_twoway;
			}
		} else {
			do control_oneway;
		}
		
	}
	
	action get_future_node {
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
			} else {future_node <- nil;}
		}
	}
	
	action get_transfer_geom {
		if (future_node != nil) {
			float alpha <- angle_between(start_node, target_node, future_node);
			float beta <- angle_between(target_node, start_node, future_node);
			if (alpha = 0) { // REVIEW IT
				transfer_geom <- polyline([target_node - {1.5*ROAD_WIDTH, 0}, target_node + {1.5*ROAD_WIDTH, 0}]) rotated_by (angle + 90);
			} else if (alpha < 180) {
				
				//ver 1 triangle with one square angle
//				point x1;
//				point x2;
//				point x3;
//				point x4; 
//				
//				float k <- ROAD_WIDTH/sin(180 - beta);
//				x2 <- target_node;
//				float a <- (start_node - target_node).x;
//				float b <- (start_node - target_node).y;
//				float d <- distance_to(target_node, start_node);
//				point x1 <- x2 + {k*a/d, k*b/d};
//				
//				float A <- (future_node - target_node).x;
//				float B <- (future_node - target_node).y;
//				float D <- distance_to(future_node, target_node);
//				point x3 <- x2 + {k*A/D, k*B/D};
//				
//				point center <- (x1 + x3)/2;
//				point x4 <- center*2 - x2;
//				point newpoint <- x2*2 - x4;
//				transfer_geom <- polyline([x2,newpoint]);

				//ver 2 triangle without square angle
				point x1;
				point x2;
				point x3;
				point x4; 
				
				float k <- ROAD_WIDTH/sin(180 - beta);
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
				point newpoint <- x2*2 - x4;
				
				point x5;
				float c <- (future_node - target_node).location.x;
				float d <- (future_node - target_node).location.y;    	
				point x5 <- target_node + { -d*ROAD_WIDTH/sqrt(c*c + d*d), c*ROAD_WIDTH/sqrt(c*c + d*d)};
				
				point x6;
				float beta <- angle_between(target_node, start_node, future_node);
				beta <- beta - 180;
				if (beta > 90) {beta <- 180 - beta;}
				float Dis <- ROAD_WIDTH/sin(beta);
				float e <- (start_node - target_node).x;
				float f <- (start_node - target_node).y;
				float distance <- distance_to(target_node, start_node);
				point x6 <- target_node + {Dis*e/distance, Dis*f/distance};

				// ver3 parallel
				point x7 <- (x2 + newpoint) - x6;
//				transfer_geom <- polygon([x2, x6, newpoint, x7]);
//				transfer_geom <- polygon([x2, newpoint, x7]);

				point x8 <- (x2 + x6)/2;
				point center <- (x2 + newpoint)/2;
				transfer_geom <- polygon([x2, x8, center, newpoint, x7]);

				if (angle_between(target_node, start_node, future_node) < 200) {
					transfer_geom <- polyline([x2, newpoint]);
				} else {
					transfer_geom <- polygon([x2, x8, center, newpoint, x7]);
				}

				//ver4 square line
//				point x1;
//				point x2;
//				point x3;
//				point x4; 
//				
//				float k <- ROAD_WIDTH/sin(180 - beta);
//				x2 <- target_node;
//				float a <- (start_node - target_node).x;
//				float b <- (start_node - target_node).y;
//				float d <- distance_to(target_node, start_node);
//				point x1 <- x2 + {k*a/d, k*b/d};
//				
//				float A <- (future_node - target_node).x;
//				float B <- (future_node - target_node).y;
//				float D <- distance_to(future_node, target_node);
//				point x3 <- x2 + {k*A/D, k*B/D};
//				
//				point center <- (x1 + x3)/2;
//				point x4 <- center*2 - x2;
//				point newpoint <- x2*2 - x4;
//				
//				float gamma <- angle_between(target_node, newpoint, start_node);
//				write gamma;
//				float d1 <- distance_to(target_node, newpoint)*cos(gamma);
//				float g <- (start_node - target_node).x;
//				float h <- (start_node - target_node).y;
//				point y1 <- target_node + {d1*a/d, d1*b/d};
//				
//				transfer_geom <- polygon([newpoint, y1]);
			} else if (alpha > 180) {
				float a <- (target_node - start_node).location.x;
				float b <- (target_node - start_node).location.y;    	
				point x <- target_node + { -b*ROAD_WIDTH/sqrt(a*a + b*b), a*ROAD_WIDTH/sqrt(a*a + b*b)};
				transfer_geom <- polyline([target_node,x]);
			}
		} else { // future_node = nil
			float a <- (target_node - start_node).location.x;
			float b <- (target_node - start_node).location.y;    	
			point x <- target_node + { -b*ROAD_WIDTH/sqrt(a*a + b*b), a*ROAD_WIDTH/sqrt(a*a + b*b)};
			transfer_geom <- polyline([target_node,x]);
		}
	}
	
	action check_direction {
		check_go_straight <- (length(vehicle_front) = 0  and check_is_on_road(front)) ? true : false;
		check_turn_left <- (length(vehicle_left) = 0  and (left.location overlaps road_belong.geom_display)) ? true : false;
		check_turn_right <- (length(vehicle_right) = 0  and (right.location overlaps road_belong.geom_display)) ? true : false;
	}
	action detect_obstacle {
		vehicle_front <- (vehicle at_distance(distance_check)) where (each.current overlaps self.front);
		vehicle_left <- (vehicle at_distance(distance_check)) where (each.current overlaps self.left);
		vehicle_right <- (vehicle at_distance(distance_check)) where (each.current overlaps self.right);
	}
	
	action observe_obstacle {
		do detect_obstacle;
		do check_direction;
	}
	
	action change_road {
		do change_segment;
		do update_polygon;
		do check_direction;	
		do get_future_node;
		do get_transfer_geom;
		is_transferred <- false;
	}
	
	action handle_right_corner {
		if (is_transferred = false) {
			point x1;
			point x2;
			point x3;
			point x4; 
			
			float beta <- angle_between(target_node, start_node, future_node);
			float k <- ROAD_WIDTH/sin(180 - beta);
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
			point newpoint <- x2*2 - x4;
			
			float gamma <- angle_between(target_node, newpoint, start_node);
			float d1 <- distance_to(target_node, newpoint)*cos(gamma);
			float g <- (start_node - target_node).x;
			float h <- (start_node - target_node).y;
			point y1 <- target_node + {d1*a/d, d1*b/d};
			
			float d2 <- distance_to(target_node, future_node);
			float i <- (future_node - target_node).x;
			float j <- (future_node - target_node).y;
			point y2 <- target_node + {d1*i/d2, d1*j/d2};
			
			transfer_geom <- polygon([newpoint, y2]);
			
			is_transferred <- true;
			// update angle, nodes
			angle <- angle_between(y1, y1 + {10,0}, y2);
			start_node <- y1;
		} else {
			do change_road;
		}
	}
	
	action handle_left_corner {
		if (is_transferred = false) {
			point x1;
			point x2;
			point x3;
			x1 <- target_node;
			//x2
			float a <- (target_node - start_node).location.x;
			float b <- (target_node - start_node).location.y;    	
			point x2 <- x1 + { -b*ROAD_WIDTH/sqrt(a*a + b*b), a*ROAD_WIDTH/sqrt(a*a + b*b)};
//			x2 <- target_node == transfer_goem.points[0]? transfer_goem.points[1]: transfer_goem.points[0];
			//x3
			float c <- (future_node - target_node).location.x;
			float d <- (future_node - target_node).location.y;    	
			point x3 <- target_node + { -d*ROAD_WIDTH/sqrt(c*c + d*d), c*ROAD_WIDTH/sqrt(c*c + d*d)};
			
			transfer_geom <- polygon([x1,x3]);
			is_transferred <- true;
			// update angle, nodes
			angle <- angle_between(x2, x2 + {10,0}, x3);
			start_node <- (x1 + x2 - x3);
			
//			if (angle_between(target_node, future_node, location) <= 180) {
//				transfer_geom <- polygon([x1,x3]);
//				is_transferred <- true;
//				// update angle, nodes
//				angle <- angle_between(x2, x2 + {10,0}, x3);
//				start_node <- (x1 + x2 - x3);
//			} else {
//				transfer_geom <- polygon([x1,x3]);
//				is_transferred <- true;
//			}
		} else {
			do change_road;
		}
	}
	action check_change_road {
		if future_node = nil {
			if (current overlaps transfer_geom) {
				do die;
			}
		} else {
			if (angle_between(start_node, target_node, future_node) > 180) and (current overlaps transfer_geom) {
				do handle_left_corner;
			} else if (angle_between(start_node, target_node, future_node) < 180) and (angle_between(target_node, start_node, future_node) > 200)
			 and (location overlaps transfer_geom) {
				//ver 1
//				point x1;
//				point x2;
//				point x3;
//				point x4; 
//				
//				float beta <- angle_between(target_node, start_node, future_node);
//				float k <- ROAD_WIDTH/sin(180 - beta);
//				x2 <- target_node;
//				float a <- (start_node - target_node).x;
//				float b <- (start_node - target_node).y;
//				float d <- distance_to(target_node, start_node);
//				point x1 <- x2 + {k*a/d, k*b/d};
//				
//				float A <- (future_node - target_node).x;
//				float B <- (future_node - target_node).y;
//				float D <- distance_to(future_node, target_node);
//				point x3 <- x2 + {k*A/D, k*B/D};
//				
//				point center <- (x1 + x3)/2;
//				point x4 <- center*2 - x2;
//				point newpoint <- x2*2 - x4;
//				if (current overlaps polyline([transfer_geom.points[0],transfer_geom.points[2]])) {
//					do change_road;
//				}
				
////				transfer_geom <- polyline([transfer_geom.points[0],transfer_geom.points[2]]);
////				is_transferred <- true;
////				// update angle, nodes
////				angle <- angle_between(transfer_geom.points[1], transfer_geom.points[1] + {10,0}, transfer_geom.points[2]);
////				start_node <- transfer_geom.points[1];
				
				do change_road;
//				do handle_right_corner;
			} else if (angle_between(start_node, target_node, future_node) < 180) and (angle_between(target_node, start_node, future_node) < 200)
			 and (current overlaps transfer_geom){
				do change_road;
			} else if (angle_between(start_node, target_node, future_node) = 0) and (current overlaps transfer_geom) {
				do change_road;
			}
		}
		
//		// action change node for twoway road
//		if (current overlaps transfer_line) { 
//			if future_node = nil {
//				do die;
//			} else {
//				// will turn left
//			    if (is_transferred = false) and (angle_between(start_node, target_node, future_node) > 180) {
//			    	point x1;
//					point x2;
//					point x3;
//					x1 <- target_node;
//					//x2
//					float a <- (target_node - start_node).location.x;
//					float b <- (target_node - start_node).location.y;    	
//					point x2 <- x1 + { -b*ROAD_WIDTH/sqrt(a*a + b*b), a*ROAD_WIDTH/sqrt(a*a + b*b)};
//					transfer_line <- polyline([x1,x2]);
//					//x3
//					float c <- (future_node - target_node).location.x;
//					float d <- (future_node - target_node).location.y;    	
//					point x3 <- target_node + { -d*ROAD_WIDTH/sqrt(c*c + d*d), c*ROAD_WIDTH/sqrt(c*c + d*d)};
//					transfer_line <- polygon([x1,x3]);
//					is_transferred <- true;
//					// update angle, nodes
//					angle <- angle_between(x2, x2 + {10,0}, x3);
//					start_node <- (x1 + x2);
//			    } else {
//			    	do change_segment;
//					do update_polygon;
//					do check_direction;	
//					do get_future_node;
//					do get_transfer_goem;
//					is_transferred <- false;
//			    }
//			}
//		} 

	}
	
	reflex move {
//		do check_change_road;
//		do observe_obstacle;
//		do define_new_target;
//		do goto target: target speed:speed;
//		do update_polygon;

		do check_change_road;
		do observe_obstacle;
		do define_new_target;
		do goto target: target speed:speed;
		do update_polygon;
	}
	
	aspect base {
		if display_polygon {
			draw current color: #yellow;
			draw front color: #red;
			draw left color: #blue;
			draw right color: #blue;
			draw transfer_geom color:#red;
		}
		
		if (type = 'CAR') {
			draw IMAGES[0] size: {length, width} rotate:heading;
		} else if (type = 'MOTORBIKE') {
			draw IMAGES[1] size: {length, width} rotate:heading;
		} else if (type = 'BUS'){
			draw IMAGES[2] size: {length, width} rotate:heading;
		}
	}
}