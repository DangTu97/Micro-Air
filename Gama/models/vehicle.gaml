/***
* Name: vehicle
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model vehicle
import "global_variables.gaml"

/* Insert your model definition here */

global { graph road_network; }

species roadNode  {
	bool is_traffic_signal;
	int time_to_change <- time_to_change;
	int counter <- rnd (time_to_change);
	rgb my_color;
	
	aspect base {
		draw shape at:location color:#red;
	}
}

species road { 
	bool is_twoway;
	geometry geom_display;
	
	aspect base {    
		draw geom_display border: #grey  color: #white ;
		if is_twoway {
			draw shape color:#grey;
		}
	}  
}


species space {
	geometry geom_display1;
	geometry geom_display2;
	aspect default {
		draw geom_display1 color:#green;
		draw geom_display2 color:#green;
	}
}

species vehicle skills:[moving] {
	string name;
	float length;
	float width;
	float df;
	float db;
	float dx; // (db + width/2)
	float dy; // (df + lenght/2)
	float max_speed;
	float prob; // probability to go opposite road
	
	point final_node;
	point source_node;
	point target;
	
	road road_belong;
	list<point> road_belong_nodes; //with direction
	point start_node;
	point target_node;
	float angle;
	list<road> shortest_path;
	
	geometry current;
	geometry front;
	geometry left;
	geometry right;
	
	list<vehicle> vehicle_front;
	list<vehicle> vehicle_left;
	list<vehicle> vehicle_right;
	
	bool display_polygon;
	bool check_go_straight;
	bool check_turn_left;
	bool check_turn_right;
	bool on_right_side;
	
	geometry target_space;
	
	bool is_on_road(geometry geom) {
		bool is_on_road <- false;
		ask road {
			if (geom.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return is_on_road;
	}
	
	action get_side {
		if (angle_between(start_node, target_node, location) < 90 and angle_between(start_node, target_node, location) > 0) {
			on_right_side <- true;
		} else {
			on_right_side <- false;
		}
	}
	
	action get_vehicle {
		vehicle_front <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.front);
		vehicle_left <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.left);
		vehicle_right <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.right);
	}
	
	action check_direction {
		check_go_straight <- (length(vehicle_front) = 0  and is_on_road(front)) ? true : false;
		check_turn_left <- (length(vehicle_left) = 0  and is_on_road(left)) ? true : false;
		check_turn_right <- (length(vehicle_right) = 0  and is_on_road(right)) ? true : false;
	}
	
	action update_polygon {
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
						  location + {- dy, dx}, location + {-dy, -dx}]) rotated_by heading;
	    
	    
	    float d <- distance_to(start_node, target_node);
		float a <- (target_node - start_node).location.x;
		float b <- (target_node - start_node).location.y;    
								  
		float size <- (speed/max_speed);
	    //ddy: dynamic dy of front polygon
		float ddy <- minimun_polygon_size*dy + size*dy;
		float k1 <- (1.0 + minimun_polygon_size + size)*dy/d;
		float k2 <- - (1.0 + minimun_polygon_size + size)*dy/d;
		
		point p1 <- location + {k1*a, k1*b};
		point p2 <- location + {k2*a, k2*b};
		point front_point;
		float angle <- angle_between(start_node,start_node + {10,0}, target_node);
		// front location with front polygon of size (dx,dy)
		if ( abs(angle_between(location, location + {10,0}, p1) - angle) < 90 ) {
			front_point <- p1;
		} else {
			front_point <- p2;
		}
		
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
						  
		target <- front.location;
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

	action speed_up {
		speed <- min(max_speed, speed + acceleration_factor);
	}
	
	action slow_down(vehicle vehicle_ahead) {
		speed <- max(0.0, speed - deceleration_factor);
	}
	
	action compute_road_belong_nodes {
//		write first(road_belong.shape.points);
//		write last(road_belong.shape.points);
//		if distance_to(location, first(road_belong.shape.points)) < distance_to(location, last(road_belong.shape.points)) {
		if (start_node = first(road_belong.shape.points)) {
			road_belong_nodes <- road_belong.shape.points;
		} else if (start_node = last(road_belong.shape.points)) {
			road_belong_nodes <- reverse(road_belong.shape.points);
		}
	}
	
	action change_node {
		start_node <- target_node;
		int idx <- (road_belong_nodes index_of target_node);
		if (idx < length(road_belong_nodes) - 1) {
			target_node <- road_belong_nodes[idx + 1];
		} else {
			road_belong <- get_next_road();
	
			if (road_belong = nil) {
				do die;
			} else {
				do compute_road_belong_nodes;
				target_node <- road_belong_nodes[1];
				
			}
		}
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
		if (target_node != nil) {
			target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
		}
	}
	
	road get_next_road {
		road next_road;
		int index <- (shortest_path index_of road_belong);
		return (index < length(shortest_path) - 1) ? shortest_path[index + 1] : nil;
	}
	
	bool check_right_side(geometry gemo) {
		if (angle_between(start_node, target_node, gemo.location) < 180 and angle_between(start_node, target_node, gemo.location) > 0) {
			return true;
		} else {
			return false;
		}
	}
	
	action control_oneway {
		if (check_go_straight = true) {
			do speed_up;
			target <- front.location;
		} else if (check_turn_left = true) {
			// turn teft
			do speed_up;
			target <- left.location;
		} else if (check_turn_right = true) {
			do speed_up;
			target <- right.location;
		} else {
			target <- front.location;
			do slow_down(first(vehicle_front));
		}
	}
	
	action control_twoway {
		do get_side;
		if (on_right_side = false) { // if on left side
			if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if ((check_turn_left = true) and (first(vehicle_front) != nil) 
			and first(vehicle_front).target_node = self.target_node) {
//				do speed_up;
//				target <- flip(prob) ? left.location : front.location;
				if flip(prob) {
					do speed_up;
					target <- left.location;
				}
			} else {
				target <- right.location;
				speed <- max(speed - deceleration_factor, 0.02);
			}
		}
		
		else { // if on right side
			if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if (check_turn_left = true) {
				if check_right_side(left) = true {
					// turn teft
					do speed_up;
					target <- left.location;
				} else if flip(prob) {
					do speed_up;
					target <- left.location;
				} else {
					target <- front.location;
					do slow_down(first(vehicle_front));
				}
			} else if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else {
				target <- front.location;
				do slow_down(first(vehicle_front));
			}
		}
	}
	
	action control_twoway_ver2 {
		if ( distance_to(location, target_node) < 20.0) {
			target <- front.location;
			if (check_go_straight = true) {
				do speed_up;
			} else {
				do slow_down(first(vehicle_front));
			}
		} else {
			do get_side;
			if (on_right_side = false) { // if on left side
				if (check_turn_right = true) {
					do speed_up;
					target <- right.location;
				} else if (check_go_straight = true) {
					do speed_up;
					target <- front.location;
				} else if ((check_turn_left = true) and (first(vehicle_front) != nil) 
				and first(vehicle_front).target_node = self.target_node) {
	//				do speed_up;
	//				target <- flip(prob) ? left.location : front.location;
					if flip(prob) {
						do speed_up;
						target <- left.location;
					}
				} else {
					target <- right.location;
					speed <- max(speed - deceleration_factor, 0.02);
				}
			}
			
			else { // if on right side
				if (check_go_straight = true) {
					do speed_up;
					target <- front.location;
				} else if (check_turn_left = true) {
					if check_right_side(left) = true {
						// turn teft
						do speed_up;
						target <- left.location;
					} else if flip(prob) {
						do speed_up;
						target <- left.location;
					} else {
						target <- front.location;
						do slow_down(first(vehicle_front));
					}
				} else if (check_turn_right = true) {
					do speed_up;
					target <- right.location;
				} else {
					target <- front.location;
					do slow_down(first(vehicle_front));
				}
			}
		}
	}
	
//	bool check_valid_turn {
//		point point1 <- target_node;
//		point point2;
//		int idx <- (road_belong_nodes index_of target_node);
//		if (idx < length(road_belong_nodes) - 1) {
//			point2  <- road_belong_nodes[idx + 1];
//		} else {
//			road my_road <- get_next_road();
//			if (my_road != nil) {
//				do die;
//			} else {
//				if (point1 = first(my_road.shape.points)) {
//					point2 <- my_road.shape.points[1];
//				} else if (point1 = last(my_road.shape.points)) {
//					point2 <- reverse(my_road.shape.points)[1];
//				}
//			}
//		}
//		
//		geometry front_space;
//		
//		angle <- angle_between(point1, point1 + {10,0}, point2);
//		
//	    float d <- distance_to(point1, point2);
//		float a <- (point2 - point1).location.x;
//		float b <- (point2 - point1).location.y;    
//								  
//		float size <- (speed/max_speed);
//	    //ddy: dynamic dy of front polygon
//		float ddy <- minimun_polygon_size*dy + size*dy;
//		float k1 <- (1.0 + minimun_polygon_size + size)*dy/d;
//		float k2 <- - (1.0 + minimun_polygon_size + size)*dy/d;
//		
//		point p1 <- location + {k1*a, k1*b};
//		point p2 <- location + {k2*a, k2*b};
//		point front_point;
//		float angle <- angle_between(start_node,start_node + {10,0}, target_node);
//		// front location with front polygon of size (dx,dy)
//		if ( abs(angle_between(location, location + {10,0}, p1) - angle) < 90 ) {
//			front_point <- p1;
//		} else {
//			front_point <- p2;
//		}
//		
//		front_space <- polygon([front_point + {ddy, -dx}, front_point + {ddy, dx},
//						  front_point + {- ddy, dx}, front_point + {-ddy, -dx}]) rotated_by angle;	
//		bool value <- false;
//		if check_right_side(front_space) {
//			value <- true;
//		}
//		return value;
//	}
	
	// different moving skill
	reflex move  {
		do get_vehicle;
		do check_direction;
		
//		if (distance_to(location, target_node) <= distance_check) {
//		if (current overlaps target_space) {
//			do change_node;
//			do update_polygon;
//		}

		if (current overlaps target_space) {
			point p1 <- start_node;
			point p2 <- target_node;
			road my_road <- road_belong;
			do change_node;
			do update_polygon;
			
			if check_right_side(front) = false {
				start_node <- p1;
				target_node <- p2;
				angle <- angle_between(start_node, start_node + {10,0}, target_node);
				road_belong <- my_road;
				do update_polygon;
				target_space <- polyline([target_node - {1.5*road_width, 0}, target_node + {1.5*road_width, 0}]) rotated_by (angle + 90);
			}
		}
		
		if road_belong.is_twoway = true {
			do control_twoway;
		} else {
			do control_oneway;
		}
		
		do goto target: target speed:speed;
		do update_polygon;
	}
	
	aspect base {
		if display_polygon {
			draw current color: #yellow;
			draw front color: #red;
			draw left color: #blue;
			draw right color: #blue;
			draw target_space color:#red;
		}
		
		if (name = 'car') {
			draw images[0] size: {length, width} rotate:heading;
		}
		
		if (name = 'motorbike') {
			draw images[1] size: {length, width} rotate:heading;
		}
	}
}