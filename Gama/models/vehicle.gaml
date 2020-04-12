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
	int time_to_change <- 300;
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
	float dx;
	float dy;
	float max_speed;
	bool display_polygon;
	
	float prob; // probability to go opposite road
	road road_belong;
	
	point final_node;
	point source_node;
	point next_node;
	point current_node;
	point target;
	list<point> shortest_path;
	
	geometry current;
	geometry front;
	geometry left;
	geometry right;
	
	list<vehicle> vehicle_front;
	list<vehicle> vehicle_left;
	list<vehicle> vehicle_right;
	
	list<vehicle> get_vehicle_conflict_front {
		list<vehicle> vehicle_conflict_front <- (vehicle at_distance(12 + dy)) where (each.current overlaps self.front);
		return vehicle_conflict_front;
	}
	
	list<vehicle> get_vehicle_conflict_left {
		list<vehicle> vehicle_conflict_left <- (vehicle at_distance(12 + dy)) where (each.current overlaps self.left);
		return vehicle_conflict_left;
	}
	
	list<vehicle> get_vehicle_conflict_right {
		list<vehicle> vehicle_conflict_right <- (vehicle at_distance(12 + dy)) where (each.current overlaps self.right);
		return vehicle_conflict_right;
	}
	
	bool check_go_straight {
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
		bool is_on_road <- false;
		ask road {
			if (myself.front.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return (length(vehicle_conflict_front) = 0  and is_on_road) ? true : false;
//		return (length(vehicle_conflict_front) = 0) ? true : false;
	}
	
	bool check_turn_left {
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		bool is_on_road <- false;
		ask road {
			if (myself.left.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return ((length(vehicle_conflict_left) = 0) and is_on_road) ? true : false;
	}
	
	bool check_turn_right {
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		bool is_on_road <- false;
		ask road {
			if (myself.right.location overlaps self.geom_display){
				is_on_road <- true;
			}
		}
		return ((length(vehicle_conflict_right) = 0) and is_on_road) ? true : false;
	}
	
	action update_polygon {
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
						  location + {- dy, dx}, location + {-dy, -dx}]) rotated_by heading;
	    
	    float d <- distance_to(current_node, next_node);
		float a <- (next_node - current_node).location.x;
		float b <- (next_node - current_node).location.y;    
								  
		float size <- (speed/max_speed);
	    //ddy: dynamic dy of front polygon
		float ddy <- 0.1*dy + size*dy;
		float k1 <- (1.0 + 0.1 + size)*dy/d;
		float k2 <- - (1.0 + 0.1 + size)*dy/d;
		
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
		
		left <- polygon([left_point + {ddy, -0.5*dx}, left_point + {ddy, 0.5*dx},
						  left_point + {- ddy, 0.5*dx}, left_point + {-ddy, -0.5*dx}]) rotated_by angle;
		
		// right location with right polygon of size (dx,dy)
		point right_point <- front_point + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
			  
		right <- polygon([right_point + {ddy, -0.5*dx}, right_point + {ddy, 0.5*dx},
						  right_point + {- ddy, 0.5*dx}, right_point + {-ddy, -0.5*dx}]) rotated_by angle;
						  
		target <- front.location;
	}
	
	action compute_shortest_path {
   		path my_path <- path_between(road_network, source_node, final_node);
        list my_path <- string(my_path) split_with('::[]()as path');
       
    	loop p over:my_path {
    		list a <- string(p) split_with('{}');
    		shortest_path <+ point('{' + a[0] + '}');
    	}
	}

	action speed_up {
		speed <- min(max_speed, speed + 0.05);
	}
	
	action slow_down(vehicle vehicle_ahead) {
		speed <- max(0.0, speed - 0.4);
	}
	
//	reflex move when:target != nil {
//		if (distance_to(location, next_node) < 6) {
//			// when vehicle reaches final destination, do die
//			if (next_node = final_node) {
//				do die;
//			}
//			
//			// assign new next_node, current_node when vehicle reaches a node which is not the final destination
//			roadNode node;
//			ask roadNode {
//				if (distance_to(myself.next_node, self.location) < 1) {
//					node <- self;
//				}   
//			}
//			if ((location overlaps node.shape) and (node.is_traffic_signal = true)) {
//				int index <- (shortest_path index_of next_node);
//				if (index < length(shortest_path) - 1) {
//					current_node <- next_node;
//					next_node <- shortest_path[index + 1];
//				} else {
//					next_node <- nil;
//					do die;
//				}
//			}
//		}
//		
//		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
//		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
//		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
//		
//		if (speed > max_speed) {
//			speed <- max_speed;
//		}
//
//		if (check_go_straight() = true) {
//			do speed_up;
//			target <- front.location;
//		} else if (check_turn_left() = true) {
//			do speed_up;
//			target <- left.location;
//		} else if (check_turn_right() = true) {
//			do speed_up;
//			target <- right.location;
//		} else {
//			target <- front.location;
//			do slow_down(first(get_vehicle_conflict_front()));
//		}
//		do goto target: target speed:speed;
//	}
	
	// different moving skill
	reflex move when:target != nil {
		if (distance_to(location, next_node) < 6) {
			// when vehicle reaches final destination, do die
			if (next_node = final_node) {
				do die;
			}
			// assign new next_node, current_node when vehicle reaches a node which is not the final destination
			roadNode node;
			ask roadNode {
				if (distance_to(myself.next_node, self.location) < 1) {
					node <- self;
				}   
			}
			
			if (node.is_traffic_signal = true) and (node.my_color = #red) {
				speed <- 0.0;
			}
			
			if ((location overlaps node.shape) and (node.my_color = #green)) {
				int index <- (shortest_path index_of next_node);
				if (index < length(shortest_path) - 1) {
					current_node <- next_node;
					next_node <- shortest_path[index + 1];
					road_belong <- first(road where ((each.start = current_node) 
										 and (each.end = next_node)));
				} else {
					next_node <- nil;
					do die;
				}
			}
		}
		
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		
		vehicle_front <- get_vehicle_conflict_front();
		vehicle_left <- get_vehicle_conflict_left();
		vehicle_right <- get_vehicle_conflict_right();
		
		if (road_belong.linked_road != nil) {
			if (location overlaps road_belong.linked_road.geom_display) {
				//turn right if it can
				if (check_turn_right() = true) {
					do speed_up;
					target <- right.location;
				} else if (check_go_straight() = true) {
					do speed_up;
					target <- front.location;
				} else if ( (check_turn_left() = true) and 
					first(get_vehicle_conflict_front()).road_belong.angle = self.road_belong.angle) {
					do speed_up;
					target <- flip(prob) ? left.location : front.location;
				} else {
					target <- right.location;
					speed <- max(speed - 0.4, 0.02);
//					write speed;
//					do slow_down(first(get_vehicle_conflict_front()));
				}
			}
		}
		
		if (location overlaps road_belong.geom_display) {
			if (check_go_straight() = true) {
				do speed_up;
				target <- front.location;
			} else if (check_turn_left() = true) {
				// turn teft
				if (left.location overlaps road_belong.geom_display) {
					do speed_up;
					target <- left.location;
				} else {
					if (road_belong.linked_road != nil) {
						target <- flip(prob) ? left.location : front.location;
					}
				}
			} else if (check_turn_right() = true) {
				do speed_up;
				target <- right.location;
			} else {
				if (first(get_vehicle_conflict_front()).road_belong.angle = self.road_belong.angle){
					target <- front.location;
					do slow_down(first(get_vehicle_conflict_front()));
				} else {
					target <- right.location;
					speed <- max(speed - 0.4, 0.02);
				}
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


