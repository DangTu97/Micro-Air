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

species traffic_light  {
	bool is_traffic_signal;
	int counter;
	bool is_green;
	bool is_yellow;
	geometry my_geom;
	float direction_control;
	
	reflex dynamic when: is_traffic_signal {
		counter <- counter + 1;
        if (counter >= green_time + red_time + yellow_time) { 
            counter <- 0;
        } 
        if (counter <= green_time) {
        	is_green <- true;
        	is_yellow <- false;
        } else if (counter > green_time and counter <= green_time + yellow_time) {
        	is_green <- false;
        	is_yellow <- true;
        }
        else {
        	is_green <- false;
        	is_yellow <- false;
        }
	}
	aspect base {
		draw shape at:location color:is_green ? #green : (is_yellow ? #yellow : #red);
		draw my_geom color:#blue;
	}
}

species road { 
	traffic_light light_belong;
	bool is_twoway;
	geometry geom_display;
	rgb color <- #white;
	aspect base {    
		draw geom_display color: color border:#white ;
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
	geometry free_space;
	
	// more polygons for motorbike
	geometry left2;
	geometry right2;
	geometry back;
	
	bool check_turn_left2;
	bool check_turn_right2;
	bool check_back;
	list<vehicle> vehicle_left2;
	list<vehicle> vehicle_right2;
	list<vehicle> vehicle_back;
	
	int count;
	
	bool is_on_road(geometry geom) {
		bool is_on_road <- false;
		ask road {
//			if ((geom.location overlaps self.geom_display) and (self = myself.road_belong)) {
			if (geom.location overlaps self.geom_display) {
				is_on_road <- true;
			}
		}
		return is_on_road;
	}
	
	action get_side {
		if (angle_between(start_node, target_node, location) < 180 and angle_between(start_node, target_node, location) > 0) {
			on_right_side <- true;
		} else {
			on_right_side <- false;
		}
	}
	
	action get_vehicle {
		vehicle_front <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.front);
		vehicle_left <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.left);
		vehicle_right <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.right);
		
		if (name = "motorbike") {
			vehicle_left2 <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.left2);
			vehicle_right2 <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.right2);
			vehicle_back <- (vehicle at_distance(vehicle_range)) where (each.current overlaps self.back);
		}
	}
	
//	action check_direction {
//		check_go_straight <- (length(vehicle_front) = 0  and is_on_road(front)) ? true : false;
//		check_turn_left <- (length(vehicle_left) = 0  and is_on_road(left)) ? true : false;
//		check_turn_right <- (length(vehicle_right) = 0  and is_on_road(right)) ? true : false;
//		
//		if (name = "motorbike") {
//			check_turn_left2 <- (length(vehicle_left2) = 0  and is_on_road(left2)) ? true : false;
//			check_turn_right2 <- (length(vehicle_right2) = 0  and is_on_road(right2)) ? true : false;
//			check_back <- (length(vehicle_back) = 0  and is_on_road(back)) ? true : false;
//		}
//	}

	action check_direction {
		check_go_straight <- (length(vehicle_front) = 0  and (front.location overlaps road_belong.geom_display)) ? true : false;
		check_turn_left <- (length(vehicle_left) = 0  and (left.location overlaps road_belong.geom_display)) ? true : false;
		check_turn_right <- (length(vehicle_right) = 0  and (right.location overlaps road_belong.geom_display)) ? true : false;
		
		if (name = "motorbike") {
			check_turn_left2 <- (length(vehicle_left2) = 0 and (left2.location overlaps road_belong.geom_display)) ? true : false;
			check_turn_right2 <- (length(vehicle_right2) = 0  and (right2.location overlaps road_belong.geom_display)) ? true : false;
			check_back <- (length(vehicle_back) = 0  and (back.location overlaps road_belong.geom_display)) ? true : false;
		}
	}
	
	action update_polygon {
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
						  location + {- dy, dx}, location + {-dy, -dx}]) rotated_by heading;
	    
	    
	    float d <- distance_to(start_node, target_node);
		float a <- (target_node - start_node).location.x;
		float b <- (target_node - start_node).location.y;    	
		point front_point;	
						  
		float size <- (speed/max_speed);
//	    //ddy: dynamic dy of front polygon
		float ddy <- minimun_polygon_size*dy + size*dy;
		
//		float k1 <- (1.0 + minimun_polygon_size + size)*dy/d;
//		float k2 <- - (1.0 + minimun_polygon_size + size)*dy/d;	
//		float angle <- angle_between(start_node,start_node + {10,0}, target_node);
//		point p1 <- location + {k1*a, k1*b};
//		point p2 <- location + {k2*a, k2*b};
//		// front location with front polygon of size (dx,dy)
//		if ( abs(angle_between(location, location + {10,0}, p1) - angle) < 90 ) {
//			front_point <- p1;
//		} else {
//			front_point <- p2;
//		}
		
		float h <- (1.0 + minimun_polygon_size + size)*dy;
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
						  
		target <- front.location;
		
		// more geometry for motorbike
		if (name = 'motorbike') {
			float D2 <- 2.5*dx;
			point left2_point <- front_point + {b*D2/sqrt(a*a + b*b), - a*D2/sqrt(a*a + b*b)};
			left2 <- polygon([left2_point + {ddy, -polygon_width_size*dx}, left2_point + {ddy, polygon_width_size*dx},
							  left2_point + {- ddy, polygon_width_size*dx}, left2_point + {-ddy, -polygon_width_size*dx}]) rotated_by angle;
							  
			point right2_point <- front_point + {-b*D2/sqrt(a*a + b*b), a*D2/sqrt(a*a + b*b)};	
			right2 <- polygon([right2_point + {ddy, -polygon_width_size*dx}, right2_point + {ddy, polygon_width_size*dx},
							  right2_point + {- ddy, polygon_width_size*dx}, right2_point + {-ddy, -polygon_width_size*dx}]) rotated_by angle;
			
			point back_point;
			float my_angle <- angle_between(target_node, target_node + {10, 0}, start_node);
			float h <- (1.0 + minimun_polygon_size + size)*dy;
			float bxx <- h*cos(my_angle);
			float byy <- h*sin(my_angle);
			back_point <- location + {bxx, byy};
			back <- polygon([back_point + {ddy, -dx}, back_point + {ddy, dx},
						  back_point + {- ddy, dx}, back_point + {-ddy, -dx}]) rotated_by angle;	
		}
		
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
//		if (distance_to(start_node,first(road_belong.shape.points)) < 1) {
//			road_belong_nodes <- road_belong.shape.points;
//		} else {
//			road_belong_nodes <- reverse(road_belong.shape.points);
//		}

		if (start_node = first(road_belong.shape.points)) {
			road_belong_nodes <- road_belong.shape.points;
		} else if (start_node = last(road_belong.shape.points)) {
			road_belong_nodes <- reverse(road_belong.shape.points);
		}
	}
	
	action change_node {
		count <- 0;
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
					} else if (target_node = last(my_feature_road.shape.points)) {
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
	
	action control_oneway_ver2 {
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
		} else if (check_turn_left2 = true) {
			do speed_up;
			target <- left2.location;
		} else if (check_turn_right2 = true) {
			do speed_up;
			target <- right2.location;
		} else if (check_back = true) and (count < 20) {
			target <- back.location;
		}
		else {
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
			and first(vehicle_front).target_node = target_node) {
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
			} else if (check_turn_left = true) and (check_right_side(left) = true) {
				do speed_up;
				target <- left.location;
			}
			else if (check_turn_left = true) and (check_right_side(left) = false ) and flip(prob) {
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
	}
	
	action control_twoway_ver2 {
		do get_side;
		if (on_right_side = false) { // if on left side
			if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if ((check_turn_left = true) and (first(vehicle_front) != nil) 
			and first(vehicle_front).target_node = target_node) and flip(prob) {
				do speed_up;
				target <- left.location;
			} else {
				target <- right.location;
				speed <- max(speed - deceleration_factor, 0.02);
			}
		}
		
		else { // if on right side
			if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if (check_turn_left = true) and ((check_right_side(left) = true) or (check_right_side(left) = false  and flip(prob))) {
				do speed_up;
				target <- left.location;
			} else if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else if (name = "motorbike") and (check_turn_left2 = true) and ((check_right_side(left2) = true) or (check_right_side(left2) = false and flip(prob))) {
				do speed_up;
				target <- left2.location;
			} else if (name = "motorbike") and (check_turn_right2 = true) {
				do speed_up;
				target <- right2.location;
			} else if (name = "motorbike") and (check_back = true)  and (count < 20) {
				target <- back.location;
			}
			else {
				target <- front.location;
				do slow_down(first(vehicle_front));
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
	
	action follow_traffic_light {
		if (road_belong.light_belong != nil) and (front overlaps road_belong.light_belong.my_geom) and (angle = road_belong.light_belong.direction_control) 
		and  (road_belong.light_belong.is_green = false) {
			speed <- 0;
		}
	}
	
	// different moving skill
	reflex move  {
		count <- count + 1;
		do get_vehicle;
		do check_direction;
		
//		if (distance_to(location, target_node) <= distance_check) {
//		if (current.location overlaps target_space) {
//			do change_node;
//			do update_polygon;
//		}

		if (current overlaps target_space) {
			point p1 <- start_node;
			point p2 <- target_node;
			road my_road <- road_belong;
			list<point> my_nodes <- road_belong_nodes;
			geometry my_target_space <- target_space;
			do change_node;
			do update_polygon;
			do check_direction;
			if (check_right_side(front) = false) or (is_on_road(front) = false) {
				start_node <- p1;
				target_node <- p2;
				angle <- angle_between(start_node, start_node + {10,0}, target_node);
				road_belong <- my_road;
				road_belong_nodes <- my_nodes;
				target_space <- my_target_space;
				do update_polygon;
				do check_direction;
			}
		}
		
		if road_belong.is_twoway = true {
//			do control_twoway;
			do control_twoway_ver2;
		} else {
//			do control_oneway;
			do control_oneway_ver2;
		}
		
		do follow_traffic_light;
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
//			draw free_space color:#red;
			draw left2 color:#red;
			draw right2 color:#red;
			draw back color: #blue;
		}
		
		if (name = 'car') {
			draw images[0] size: {length, width} rotate:heading;
		}
		
		if (name = 'motorbike') {
			draw images[1] size: {length, width} rotate:heading;
		}
	}
}