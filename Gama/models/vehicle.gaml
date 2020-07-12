/***
* Name: vehicle
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model vehicle
import "global_variables.gaml"

/* Insert your model definition here */

global { 
	graph road_network;
	float vehicle_average_speed;
	int vehicle_counter;
	float vehicle_timer;
	string model_name;
}

species traffic_light  {
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
	float road_width;
	traffic_light light_belong;
	geometry geom_display;
	rgb color <- #white;
	
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
	float prob_go_opposite; // probability to go opposite road
	float prob_turn_right; // probability to turn right
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
	bool check_go_straight;
	bool check_turn_left;
	bool check_turn_right;
	float acceleration_factor;
	float deceleration_factor;
	
	// additional attributes
	list<vehicle> vehicle_front;
	list<vehicle> vehicle_left;
	list<vehicle> vehicle_right;
	float distance_check;
	float minimun_length_size;
	bool display_polygon;
	bool on_right_side;
	float width_size; // for right and left polygons
	
	geometry target_space;
	geometry free_space;
	
	
	// parameter
	int count;
	int timer;
	string gender;
	string age;
	
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
	
	action update_polygon {
		// polygons for vehicle
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
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
		
		left <- polygon([left_point + {ddy, -width_size*dx}, left_point + {ddy, width_size*dx},
						  left_point + {- ddy, width_size*dx}, left_point + {-ddy, -width_size*dx}]) rotated_by angle;
		
		// right location with right polygon of size (dx,dy)
		point right_point <- front_point + { -b*D/sqrt(a*a + b*b), a*D/sqrt(a*a + b*b)};	
			  
		right <- polygon([right_point + {ddy, -width_size*dx}, right_point + {ddy, width_size*dx},
						  right_point + {- ddy, width_size*dx}, right_point + {-ddy, -width_size*dx}]) rotated_by angle;
						  
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

	action accelerate {
		speed <- min(max_speed, speed + acceleration_factor);
	}
	
	action decelerate(vehicle vehicle_ahead) {
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
				if (vehicle_average_speed != nil) {
					float t <- timer / (1/STEP);
					vehicle_timer <- (vehicle_timer*vehicle_counter + t)/(vehicle_counter + 1);
					vehicle_average_speed <- 200/vehicle_timer;
					vehicle_counter <- vehicle_counter + 1;
				}
				do die;
			} else {
				do compute_road_belong_nodes;
				target_node <- road_belong_nodes[1];
			}
		}
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
		
		if (target_node != nil) {		
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
			do decelerate(first(vehicle_front));
		}
	}
	
	action control_twoway {
		do get_side;
		if (on_right_side = false) { // if on left side
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
			} else if (check_turn_left = true) and (check_right_side(left) = true) {
				do accelerate;
				target <- left.location;
			} else if (check_turn_right = true) and flip(prob_turn_right) {
				do accelerate;
				target <- right.location;
			} else if (check_turn_left = true) and (check_right_side(left) = false ) and flip(prob_go_opposite) {
				do accelerate;
				target <- left.location;
			} else {
				target <- front.location;
				do decelerate(first(vehicle_front));
			}
		}
	}
	
	// this action is for the intersection
	action observe_traffic_light {
		if ((road_belong.light_belong != nil) and (front overlaps road_belong.light_belong.my_geom) and (angle = road_belong.light_belong.direction_control)) {
			if (road_belong.light_belong.is_green = false) {
				speed <- 0;
			}
 		}
	}
	
	action control_at_intersection {
		list<list<int>> check_list <- [[270,0], [90,180], [180,270], [0,90]];
		list<vehicle> vehicle_intersection <- (vehicle at_distance(distance_check) where (([angle, each.angle] in check_list) and (each.front overlaps front)));
		check_go_straight <- (length(vehicle_intersection) > 0) ? false : check_go_straight;
		
		if (check_go_straight = true) {
			do accelerate;
			target <- front.location;
		} else if (check_turn_left = true) and (check_right_side(left) = true) {
			do accelerate;
			target <- left.location;
		} else if (check_turn_right = true) and flip(prob_turn_right) {
			do accelerate;
			target <- right.location;
		} else if (check_turn_left = true) and (check_right_side(left) = false ) and flip(prob_go_opposite) {
			do accelerate;
			target <- left.location;
		} else {
			target <- front.location;
			speed <- 0.0;
		}	
				
		if (block_space(0) overlaps front) or (block_space(0) overlaps current) {
			target <- right;
		}
	}
	
	action check_direction {
		check_go_straight <- (length(vehicle_front) = 0  and is_on_road(front)) ? true : false;
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
	
	action define_new_target {
		if road_belong.is_twoway = true {
			if distance_to(location, {100, 100}) < distance_check {
				do control_at_intersection;
			} else {
				do control_twoway;
			}
			
		} else {
			do control_oneway;
		}
		
	}
	
	action check_change_road {
//		if (distance_to(location, target_node) <= distance_check) {
//		if (current overlaps target_space) {
//			do change_node;
//			do update_polygon;
//		}

		// action change node for twoway road
		if (current overlaps target_space) {
			point p1 <- start_node;
			point p2 <- target_node;
			road my_road <- road_belong;
			list<point> my_nodes <- road_belong_nodes;
			geometry my_target_space <- target_space;
			do change_node;
			do update_polygon;
			do check_direction;
			
			// or (check_go_straight = false)
			if (check_right_side(front) = false) or (is_on_road(front) = false)  {
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
	}
	
	// moving skill
	reflex move  {
		timer <- timer + 1;
		// in study case of Giai Phong road, vehicles must decrease their speed at the end of the road 
		// due to the high density of traffic there
		
		if ((distance_to(location, target_node) <= 5) and (model_name = 'study_case')) {
			max_speed <- 2.0 + rnd(2.0);
			speed <- max_speed;
		}
		
		do check_change_road;
		do observe_obstacle;
		do define_new_target;
		do observe_traffic_light;
		do goto target: target speed:speed;
		do update_polygon;
	}
	
	aspect base {
		if display_polygon {
			draw current color: #yellow;
			draw front color: #red;
			draw left color: #blue;
			draw right color: #blue;
//			draw target_space color:#red;
//			draw free_space color:#red;
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

species block_space {
	aspect base {
		draw shape color: #blue;
	}
}

species init_space {
	geometry geom;
	point my_point;
	aspect base {
		draw geom color: #blue border:#black;
	}
}