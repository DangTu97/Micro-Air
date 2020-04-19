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

species roadNode skills: [skill_road_node] {
	bool is_traffic_signal;
	int time_to_change <- time_to_change;
	int counter <- rnd (time_to_change);
	rgb my_color;
	
	reflex dynamic when: is_traffic_signal {
		counter <- counter + 1;
		if (counter >= time_to_change) { 
			counter <- 0;
			stop[0] <- empty (stop[0]) ? roads_in : [] ;
		} 
	}
	
	reflex change_color {
		if (length(stop) > 0) {
			my_color <- (empty (stop[0]) ? #green : #red);
		}
	}
	
	aspect base {
		draw shape at:location color: (is_traffic_signal ? my_color : #white);
//		draw shape at:location color:#red;
	}
}

species road skills: [skill_road] { 
	string oneway;
	int id;
	geometry geom_display;
	road linked_road;
	float angle;
	point start;
	point end;
	
	aspect base {    
		draw geom_display border: #grey  color: #white ;
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
	road next_road;
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
	
	bool is_on_road(geometry poly) {
		bool is_on_road <- false;
		ask road {
			if (poly.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return is_on_road;
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
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
						  location + {- dy, dx}, location + {-dy, -dx}]) rotated_by heading;
	    
	    point current_node <- road_belong.start;
	    point next_node <- road_belong.end;
	    
	    float d <- distance_to(current_node, next_node);
		float a <- (next_node - current_node).location.x;
		float b <- (next_node - current_node).location.y;    
								  
		float size <- (speed/max_speed);
	    //ddy: dynamic dy of front polygon
		float ddy <- minimun_polygon_size*dy + size*dy;
		float k1 <- (1.0 + minimun_polygon_size + size)*dy/d;
		float k2 <- - (1.0 + minimun_polygon_size + size)*dy/d;
		
		point p1 <- location + {k1*a, k1*b};
		point p2 <- location + {k2*a, k2*b};
		point front_point;
		float angle <- angle_between(current_node,current_node + {10,0}, next_node);
		// front location with front polygon of size (dx,dy)
		if ( abs(angle_between(location, location + {10,0}, p1) - road_belong.angle) < 20 ) {
			front_point <- p1;
		} else {
			front_point <- p2;
		}
		
		front <- polygon([front_point + {ddy, -dx}, front_point + {ddy, dx},
						  front_point + {- ddy, dx}, front_point + {-ddy, -dx}]) rotated_by angle;			  
		
		float D <- dx;
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
//   		write road_id_list;
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
	
	action get_next_road {
		int index <- (shortest_path index_of road_belong);
		if (index < length(shortest_path) - 1) {
			next_road <- shortest_path[index + 1];
		} else {
			next_road <- nil;
		}
	}
	
	// different moving skill
	reflex move when:target != nil {
		do get_vehicle;
		do check_direction;
//		
//		if (distance_to(location, road_belong.end) < distance_check) {
//			do get_next_road;
//			if (next_road = nil) {
//				do die;
//			} else {
//				road_belong <- next_road;
//			}
//		}
		
		if next_road = nil {
			if (distance_to(location, road_belong.end) < distance_check) {
				do die;
			} 
		} else if (location overlaps next_road.geom_display) {
			road_belong <- next_road;
			do get_next_road;
		}
		
		if (road_belong.linked_road != nil) and (location overlaps road_belong.linked_road.geom_display) {
			//turn right if it can
			if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if ((check_turn_left = true) and (first(vehicle_front) != nil) and first(vehicle_front).road_belong.angle = self.road_belong.angle) {
				do speed_up;
				target <- flip(prob) ? left.location : front.location;
			} else {
				target <- right.location;
				speed <- max(speed - deceleration_factor, 0.02);
//					write speed;
//					do slow_down(first(get_vehicle_conflict_front()));
			}
		}
		
		else if (location overlaps road_belong.geom_display) {
			if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if (check_turn_left = true) {
				// turn teft
				if (left.location overlaps road_belong.geom_display) {
					do speed_up;
					target <- left.location;
				} else {
					if (road_belong.linked_road != nil) {
						target <- flip(prob) ? left.location : front.location;
					}
				}
			} else if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else {
				target <- front.location;
				do slow_down(first(vehicle_front));
			}
		} 
		 
		// when vehicle is outside of the road 
		else {
			if (check_go_straight = true) {
				do speed_up;
				target <- front.location;
			} else if (check_turn_left = true) {
				do speed_up;
				target <- left.location;
			} else if (check_turn_right = true) {
				do speed_up;
				target <- right.location;
			} else {
				speed <- 0.0;
			}
		}
		
		do goto target: target speed:speed;
	}
	
	reflex update_line {
		do update_polygon;
	}
	
	aspect base {
		if display_polygon {
			draw current color: #yellow;
			draw front color: #red;
			draw left color: #blue;
			draw right color: #blue;
		}
		
		if (name = 'car') {
			draw images[0] size: {length, width} rotate:heading;
		}
		
		if (name = 'motorbike') {
			draw images[1] size: {length, width} rotate:heading;
		}
	}
}