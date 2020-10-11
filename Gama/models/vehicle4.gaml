/***
* Name: vehicle4
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model vehicle4
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

species roadNode skills:[skill_road_node] {
	aspect base {
		draw circle(1) color:#blue;
	}
}

species road skills:[skill_road]{ 
	bool is_twoway;
	bool oneway;
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
	geometry back;
	bool check_go_straight;
	bool check_turn_left;
	bool check_turn_right;
	bool check_go_back;
	float acceleration_factor;
	float deceleration_factor;
	
	// additional attributes
	road road_belong;
	list<point> road_belong_nodes; //with direction
	list<vehicle> vehicle_front;
	list<vehicle> vehicle_left;
	list<vehicle> vehicle_right;
	list<vehicle> vehicle_back;
	float distance_check;
	float minimun_length_size;
	bool display_polygon;
	bool on_right_side;
	float polygon_width_size; // for right and left polygons
	float prob_go_opposite; // probability to go opposite road
	float prob_turn_right; // probability to turn right
	
	geometry transfer_geom;
	bool is_transferred;
	
	action compute_shortest_path {
   		path path_computed <- path_between(road_network, source_node, final_node);
   		list road_id_list <- string(path_computed) split_with('[()],road as path');
		if (length(road_id_list) > 0) {
			loop i from: 0 to:length(road_id_list) - 1 {
				shortest_path <+ road(int(road_id_list[i]));
			}
		}  
	}
	
	bool check_is_on_road(geometry geom) {
		return (geom overlaps road_belong.geom_display) ? true : false;
	}
	
	bool is_on_road(geometry geom) {
		bool is_on_road <- false;
		ask road {
			if (geom.location overlaps self.geom_display) {
				is_on_road <- true;
			}
		}
		return is_on_road;
	}
	
	bool check_on_right_side(geometry geom) {
		return (angle_between(start_node, target_node, geom.location) < 180) ? true : false;
	}
	
	action update_angle {
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
	}
	
	action accelerate {
		speed <- min(max_speed, speed + acceleration_factor);
	}
	
	action decelerate {
		speed <- max(0.0, speed - deceleration_factor);
	}
	
	road get_next_road {
		int index <- (shortest_path index_of road_belong);
		return (index < length(shortest_path) - 1) ? shortest_path[index + 1] : nil;
	}
	
	action compute_road_belong_nodes {
		if (start_node = first(road_belong.shape.points)) {
			road_belong_nodes <- road_belong.shape.points;
		} else if (start_node = last(road_belong.shape.points)) {
			road_belong_nodes <- reverse(road_belong.shape.points);
		}
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
		
//		point back_point <- (location*(3 + 2*minimun_length_size) - front_point*(1 + minimun_length_size))/(2+minimun_length_size);
		float t <- (1.0 + minimun_length_size)*dy;
		point u <- location + {t*cos(angle), t*sin(angle)};
		point back_point <- location*2 - u;  
		back <- polygon([back_point + {(minimun_length_size)*dy, -dx}, back_point + {(minimun_length_size)*dy, dx},
						 back_point + {-(minimun_length_size)*dy, dx}, back_point + {-(minimun_length_size)*dy, -dx}]) rotated_by angle;  
		
//		back <- polygon([back_point + {-1,-1}, back_point + {1,-1}, back_point + {1,1}, back_point + {-1,1}]) rotated_by angle;
		
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
	
	action control_twoway_ver2 {
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
			} else if (check_go_back = true) and (speed = 0) {
				do accelerate;
				target <- back.location;
			}
			else {
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
	
	action control_oneway_ver2 {
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
		} else if (check_go_back = true) and (speed = 0) {
			do accelerate;
			target <- back.location;
		} 
		else {
			target <- front.location;
			do decelerate();
		}
	}
	
	action define_new_target {
		if road_belong.is_twoway = true {
			if (is_transferred = true) {
				if (check_go_straight = true) {
					target <- front.location;
					do accelerate;
				} else {
					target <- front.location;
					do decelerate;
				}
			} else {
				do control_twoway;
//				do control_twoway_ver2;
			}
		} else {
			do control_oneway;
//			do control_oneway_ver2;
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
	
	action check_direction {
		check_go_straight <- (length(vehicle_front) = 0  and is_on_road(front)) ? true : false;
		check_turn_left <- (length(vehicle_left) = 0  and (left.location overlaps road_belong.geom_display)) ? true : false;
		check_turn_right <- (length(vehicle_right) = 0  and (right.location overlaps road_belong.geom_display)) ? true : false;
		check_go_back <- (length(vehicle_back) = 0  and is_on_road(back)) ? true : false;
	}
	
	action detect_obstacle {
		vehicle_front <- (vehicle at_distance(distance_check)) where (each.current overlaps self.front);
		vehicle_left <- (vehicle at_distance(distance_check)) where (each.current overlaps self.left);
		vehicle_right <- (vehicle at_distance(distance_check)) where (each.current overlaps self.right);
		vehicle_back <- (vehicle at_distance(distance_check)) where (each.current overlaps self.back);
	}
	
	action observe_obstacle {
		do detect_obstacle;
		do check_direction;
	}
	
	action get_transfer_geom {
		if (future_node != nil) {
			float alpha <- angle_between(start_node, target_node, future_node);
			float beta <- angle_between(target_node, start_node, future_node);
			
			if (alpha = 0) {
				transfer_geom <- polyline([target_node - {ROAD_WIDTH, 0}, target_node, target_node + {ROAD_WIDTH, 0}]) rotated_by (angle + 90);
			}
			else if (alpha < 180) {
				//ver 1
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
				
				float gamma <- angle_between(target_node, newpoint, start_node);
				float d1 <- distance_to(target_node, newpoint)*cos(gamma);
				float d <- distance_to(target_node, start_node);
				float g <- (start_node - target_node).x;
				float h <- (start_node - target_node).y;
				point x5 <- target_node + {d1*a/d, d1*b/d};
						
				point x6 <- x5 + x5 - newpoint;
				transfer_geom <- polyline([newpoint, x5, x6]);

				//transfer line for short segment
				if distance_to(start_node, target_node) < distance_to(x5, target_node) {
					transfer_geom <- polyline([target_node - {ROAD_WIDTH,0}, target_node, target_node + {ROAD_WIDTH,0}]) rotated_by 
					(angle+90);
				}
				
			} else if (alpha > 180) {
				float a <- (target_node - start_node).location.x;
				float b <- (target_node - start_node).location.y;    	
				point x1 <- target_node + { -b*ROAD_WIDTH/sqrt(a*a + b*b), a*ROAD_WIDTH/sqrt(a*a + b*b)};
				point x2 <- target_node + target_node - x1;
				transfer_geom <- polyline([x1, target_node,x2]);
			}
		} else {
			float a <- (target_node - start_node).location.x;
			float b <- (target_node - start_node).location.y;    	
			point x <- target_node + { -b*ROAD_WIDTH/sqrt(a*a + b*b), a*ROAD_WIDTH/sqrt(a*a + b*b)};
			point y <- target_node + target_node - x;
			transfer_geom <- polyline([x, target_node, y]);
		}
	}
	
	action define_new_segment_ver1 {
		if (check_current_segment() = false) {
			if (location overlaps road_belong.geom_display) {
				int index <- (road_belong_nodes index_of start_node);
				loop i from:index to:length(road_belong_nodes) - 2 {
					point s <- road_belong_nodes[i];
					point t <- road_belong_nodes[i+1];
					geometry l1 <- polyline([location - {ROAD_WIDTH,0}, location, location + {ROAD_WIDTH,0}]) rotated_by (angle_between(s,s+{10,0},t) + 90);
					geometry l2 <- polyline([front.location - {ROAD_WIDTH,0}, front.location, front.location + {ROAD_WIDTH,0}]) rotated_by (angle_between(s,s+{10,0},t) + 90);

					if ((l1 intersects polyline([s, t]) = true) or (l2 intersects polyline([s, t]) = true)) {
						start_node <- s;
						target_node <- t;
						angle <- angle_between(s, s + {10,0}, t);
						do update_polygon;
						do check_direction;	
						do get_future_node;
						do get_transfer_geom;
						is_transferred <- false;
					}
				}
			} 
			
			// next road case
			if ((get_next_road() != nil)) {
				road next_road <- get_next_road();
				list<point> next_road_points <- (first(next_road.shape.points) = last(road_belong_nodes)) ? next_road.shape.points : reverse(next_road.shape.points);
				if (location overlaps next_road.geom_display) {
					loop i from:0 to:length(next_road_points) - 2 {
						point s <- next_road_points[i];
						point t <- next_road_points[i+1];
						geometry l <- polyline([location - {ROAD_WIDTH,0}, location, location + {ROAD_WIDTH,0}])
						 rotated_by (angle_between(s,s+{10,0},t) + 90);
						if (l intersects polyline([s, t]) = true) {
							start_node <- s;
							target_node <- t;
							angle <- angle_between(s, s + {10,0}, t);
							do update_polygon;
							do check_direction;	
							do get_future_node;
							do get_transfer_geom;
							is_transferred <- false;
	
							road_belong <- next_road;
							road_belong_nodes <- next_road_points;
						}
					}
				} 	
			}
		}
	}
	
	action define_new_segment {
		geometry current <- polyline([start_node,target_node]) + ROAD_WIDTH;
		if (check_current_segment() = false) {
			if (location overlaps road_belong.geom_display) {
				int index <- (road_belong_nodes index_of start_node);
				loop i from:index to:length(road_belong_nodes) - 2 {
					point s <- road_belong_nodes[i];
					point t <- road_belong_nodes[i+1];
					geometry m <- polyline([s, t]) + ROAD_WIDTH;
					if (location overlaps m) {
						start_node <- s;
						target_node <- t;
						angle <- angle_between(start_node, start_node + {10,0}, target_node);
						do update_polygon;
						do check_direction;	
						do get_future_node;
						do get_transfer_geom;
						is_transferred <- false;
					}
				}
			} 
			
			if ((get_next_road() != nil)) and (location overlaps get_next_road().geom_display){
				road next_road <- get_next_road();
				list<point> next_road_points <- (first(next_road.shape.points) = last(road_belong_nodes)) ? next_road.shape.points : reverse(next_road.shape.points);
			
				loop i from:0 to:length(next_road_points) - 2 {
					point s <- next_road_points[i];
					point t <- next_road_points[i+1];
					geometry m <- polyline([s, t]) + ROAD_WIDTH;
					if (location overlaps m) {
						start_node <- s;
						target_node <- t;
						angle <- angle_between(s, s + {10,0}, t);
						do update_polygon;
						do check_direction;	
						do get_future_node;
						do get_transfer_geom;
						is_transferred <- false;
	
						road_belong <- next_road;
						road_belong_nodes <- next_road_points;
					}
				}
			}
		}
	}
	
	action handle_right_corner {
		point start <- transfer_geom.points[0];
		point end <- transfer_geom.points[2];
		point middle <- transfer_geom.points[1];
		
		if (is_transferred = false) {
			point x8 <- (start + end)/2;
			point x1 <- (start*7 + x8)/8;
			point x2 <- (start*6 + x8*2)/8;
			point x3 <- (start*5 + x8*3)/8;
			point x4 <- (start*4 + x8*4)/8;
			point x5 <- (start*3 + x8*5)/8;
			point x6 <- (start*2 + x8*6)/8;
			point x7 <- (start*1 + x8*7)/8;
			
			point y1 <- (x8*2 - x7);
			point y2 <- (x8*2 - x6);
			point y3 <- (x8*2 - x5);
			point y4 <- (x8*2 - x4);
			
			list<point> closest_list;
			road next_road <- get_next_road();
			if (next_road != nil) and (next_road.is_twoway = true)
			and ((target_node = next_road.shape.points[0] or (target_node = reverse(next_road.shape.points)[0]))) {
				closest_list <- [x1,x2,x3,x4,x5,x6,x7] closest_to (location, 1);
			} else {
				closest_list <- [x1,x2,x3,x4,x5,x6,x7,x8, y1,y2,y3,y4] closest_to (location, 1);
			}
			
			location <- closest_list[0];
//			target <- closest_list[0];
			
			//ver1
			float gamma <- angle_between(target_node, future_node, start);
			float d1 <- distance_to(target_node, start)*cos(gamma);
			float d <- distance_to(target_node, future_node);
			float g <- (future_node - target_node).x;
			float h <- (future_node - target_node).y;
			point a <- target_node + {d1*g/d, d1*h/d};
			point b <- a + a - start;
			is_transferred <- true;
			start_node <- target_node - (a - middle);
			angle <- angle_between(middle, middle + {10,0}, a);
			transfer_geom <- polyline([start, a, b]);
			
			if (current overlaps transfer_geom) {
				do change_road;
			}
		} else {
			do change_road;
			do define_new_segment;
		}
	}
	
	action handle_right_corner_ver2 {
		point start <- transfer_geom.points[0];
		point end <- transfer_geom.points[2];
		point middle <- transfer_geom.points[1];
		
		if middle = target_node {
			do change_road;
			do define_new_segment;
		} else {
			do handle_right_corner;
		}
	}
	
	action handle_left_corner {
		point start <- transfer_geom.points[0];
		point end <- transfer_geom.points[2];
		point middle <- transfer_geom.points[1];

		if (is_transferred = false) and (current overlaps polyline([middle, start])) {
			point x8 <- (start + end)/2;
			point x1 <- (start*7 + x8)/8;
			point x2 <- (start*6 + x8*2)/8;
			point x3 <- (start*5 + x8*3)/8;
			point x4 <- (start*4 + x8*4)/8;
			point x5 <- (start*3 + x8*5)/8;
			point x6 <- (start*2 + x8*6)/8;
			point x7 <- (start*1 + x8*7)/8;
			
			point y1 <- (x8*2 - x7);
			point y2 <- (x8*2 - x6);
			point y3 <- (x8*2 - x5);
			point y4 <- (x8*2 - x4);
			
			list<point> closest_list;
			road next_road <- get_next_road();
			if (next_road != nil) and (next_road.is_twoway = true)
			and ((target_node = next_road.shape.points[0] or (target_node = reverse(next_road.shape.points)[0]))) {
				closest_list <- [x1,x2,x3,x4,x5,x6,x7] closest_to (location, 1);
			} else {
				closest_list <- [x1,x2,x3,x4,x5,x6,x7,x8, y1,y2,y3,y4] closest_to (location, 1);
			}
			
			location <- closest_list[0];
//			target <- closest_list[0];
			
			float c <- (future_node - target_node).location.x;
			float d <- (future_node - target_node).location.y;    	
			point y <- target_node + { -d*ROAD_WIDTH/sqrt(c*c + d*d), c*ROAD_WIDTH/sqrt(c*c + d*d)};
			point z <- target_node + target_node - y;
			
			transfer_geom <- polyline([z,target_node,y]);
			is_transferred <- true;
			// update angle, nodes
			angle <- angle_between(start, start + {10,0}, y);
			start_node <- target_node - (y - start);
			
			if (current overlaps transfer_geom) {
				do change_road;
			}
		} else {
			do change_road;
		}
	}

	bool check_current_segment {
		geometry l <- polyline([location - {10, 0}, location, location + {10,0}]) rotated_by (angle + 90);
		return (l intersects polyline([start_node, target_node]));
	}
	
	action change_road {
		do change_segment;
		do update_polygon;
		do check_direction;	
		do get_future_node;
		do get_transfer_geom;
		is_transferred <- false;
	}
	
	action check_change_road {
		if (future_node = nil) {
			if (current overlaps transfer_geom) {
				do die;
			}
		} else {
			if (angle_between(start_node, target_node, future_node) > 180) and (current overlaps transfer_geom) {
				do handle_left_corner;
			} 
			else if (angle_between(start_node, target_node, future_node) < 180) and (current overlaps transfer_geom) {
				do handle_right_corner_ver2;
			} 
		}
	}
	
	reflex move {
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
			draw back color: #red;
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