/**
* Name: vehicle
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model vehicle
import '../global_vars.gaml'
import 'GIS_species.gaml'

/* Insert your model definition here */

global {
	graph road_network;
}

species vehicle skills:[moving] {
	//vehicle attributes
	string type;
	point source_node;
	point destination_node;
	float max_speed;
	float speed;
	float acceleration_rate;
	float deceleration_rate;
	float observing_distance;
	
	//technical attributes
	image_file image_path;
	float dx;
	float dy;
	float angle;
	point start_node;
	point target_node;
	point next_node;
	point target;
	geometry bezier_curve;
	list<road> paths;
	float width_of_road_belong;
	road road_belong;
	list<point> road_belong_nodes;
	geometry front;
	geometry left;
	geometry current;
	geometry right;
	vehicle front_vehicle_conflict;
	vehicle left_vehicle_conflict;
	vehicle right_vehicle_conflict;
	float minimum_steering_distance; 
	bool is_steering;
	bool is_passing;
	string status;
	string next_status;
	float collision_avoidance_duration;
	point direction_unit_vector;
	intersection intersection_belong;
	bool enter_conflict_space;
	
	//testing
	float reaction_time;
	bool debug;
	int delay_count;
	point orthogonal_target_point;
	
	action init_type {
		if (type = nil) { write "Please assigne vehicle type"; }
		else if (type = 'MOTORBIKE') {
			image_path <- MOTORBIKE_IMAGE;
			max_speed <- rnd(0.7*MOTORBIKE_MAXSPEED, MOTORBIKE_MAXSPEED);
			acceleration_rate <- MOTORBIKE_ACCELERATION_RATE;
			deceleration_rate <- MOTORBIKE_DECELERATION_RATE;
			dx <- MOTORBIKE_WIDTH/2 + MOTORBIKE_DB;
			dy <- MOTORBIKE_LENGTH/2 + MOTORBIKE_DF;
		}
		else if (type = 'CAR') {
			image_path <- CAR_IMAGE;
			max_speed <- rnd(0.7*CAR_MAXSPEED, CAR_MAXSPEED);
			acceleration_rate <- CAR_ACCELERATION_RATE;
			deceleration_rate <- CAR_DECELERATION_RATE;
			dx <- CAR_WIDTH/2 + CAR_DB;
			dy <- CAR_LENGTH/2 + CAR_DF;
		}
		
		//technical init
		do compute_paths;
		road_belong <- paths[0];
		width_of_road_belong <- road_belong.width;
		start_node <- source_node;
		do update_road_belong_nodes;
		target_node <- road_belong_nodes[1];
		angle <- angle_between(start_node, start_node + {10, 0}, target_node);
		heading <- angle;
		observing_distance <- COLLISION_AVOIDANCE_DURATION*speed;
		minimum_steering_distance <- max_speed*STEP #m;
		collision_avoidance_duration <- COLLISION_AVOIDANCE_DURATION;
		direction_unit_vector <- (target_node - start_node)/distance_to(target_node, start_node);
		reaction_time <- REACTION_TIME;
		do update_minimum_steering_distance;
		next_node <- get_next_node();
		//assign next_status
		roadNode r <- (target_node = road_belong.endpoints[0].location) ? road_belong.endpoints[0] : (target_node = road_belong.endpoints[1].location ? road_belong.endpoints[1] : nil);
		next_status <- (next_node = nil) ? nil : (r = nil ? 'at_corner': (r.is_intersection ? 'at_intersection' : 'at_corner'));
		
		//testing
		orthogonal_target_point <- get_orthogonal_point(start_node, target_node, 0.0, width_of_road_belong);
	}
	
	// -------------------------------- FUNCTIONS -----------------------------------
	road get_next_road {
		int index <- (paths index_of road_belong);
		return (index < length(paths) - 1) ? paths[index + 1] : nil;
	}
	
	point get_next_node {
		point future_node;
		int my_idx <- (road_belong_nodes index_of target_node);
		if (my_idx < length(road_belong_nodes) - 1) {
			future_node <- road_belong_nodes[my_idx + 1];
		} else {
			road my_future_road <- get_next_road();
			if (my_future_road != nil) {
				list<point> nodes <- my_future_road.shape.points;
				future_node <- (target_node = first(nodes)) ? nodes[1] : reverse(nodes)[1];
			} else {future_node <- nil;}
		}
		return future_node;
	}
	
	float compute_distance(point p, point p1, point p2) {
		//compute distance from p to segment of (p1, p2)
		float A <- p.x - p1.x; 
	    float B <- p.y - p1.y;
	    float C <- p2.x - p1.x; 
	    float D <- p2.y - p1.y;
	    float E <- -D; // orthogonal vector
	    float F <- C;
	    float dot <- A * E + B * F;
	    float len_sq <- E * E + F * F;
	    
	    if (dot = 0) {
//    	   	write "--";
	    }
		return abs(dot)/sqrt(len_sq);
	}
	
	point get_orthogonal_point(point s, point t, float d1, float d2) {
		// get right-hand-side orthogonal point of (s,t)
		// d1: distance from target
		// d2: distance to segment (s, t)
		float a <- (t - s).x;
		float b <- (t - s).y;    	
		point p <- t - (t - s)*d1/distance_to(s, t) + {-b*d2/sqrt(a*a + b*b), a*d2/sqrt(a*a + b*b)};
		return p;
	}

	bool prepare_to_steer {
		return (compute_distance(location, target_node, orthogonal_target_point) <= minimum_steering_distance) ? true : false;
	}
	
//	float compute_inflection(float alpha, float x, float m, float M, float road_width) {
//		if (alpha = 180) {
//			return 0.0;
//		} else {
//			float t_x <- (alpha < 180) ? 1 - x/road_width : x/road_width;
//			float t_alpha <- (alpha < 180) ? 1 - alpha/180 : alpha/180 - 1;
//			return m*t_alpha/(t_alpha + t_x) + M*t_x/(t_alpha + t_x);
//		}
//	}
	
	float compute_inflection(point t, point s, point f) {
		float alpha <- angle_between(t, s, f);
		alpha <- (alpha > 180) ? 360 - alpha : alpha;
		return (180 - alpha)/180;
	}
	
	// ------------------------------ GLOBAL ACTIONS ------------------------------
	action free_accelerate {
		speed <- min(max_speed, speed + acceleration_rate);
	}
	
//	action following_accelerate (vehicle front_vehicle) {
//		speed <- min(max_speed, speed + (front_vehicle.speed - speed)*STEP);
//	}
	
	action decelerate {
		speed <- dead(front_vehicle_conflict) ? max(0.0, speed - deceleration_rate) : 0.95*front_vehicle_conflict.speed;
//		speed <- dead(front_vehicle_conflict) ? max(0.0, speed - deceleration_rate) : front_vehicle_conflict.speed;
//		speed <- dead(front_vehicle_conflict) ? max(0.0, speed - deceleration_rate) : front_vehicle_conflict.real_speed;
	}
	
	action compute_paths {
		path paths_computed <- path_between(road_network, source_node, destination_node);
		paths <- paths_computed.edges as list<road>;
		if (length(paths) = 0) {
			write "Source node and final node are the same, force to die";
			do die;
		}
	}
	
	action update_road_belong_nodes {
		road_belong_nodes <- (start_node = first(road_belong.shape.points)) ? road_belong.shape.points : reverse(road_belong.shape.points);
	}
	
	action update_space {
		point front_center;
		point right_center;
		point left_center;
		float h;
		float alpha;
		if (is_steering or is_passing) { 
			h <- 0.75*dy;
			front_center <- location + {(dy + h)*cos(heading), (dy + h)*sin(heading)}; 
			alpha <- heading;
		} else {
			h <- 0.1*dy + (speed/max_speed)*2*dy;
			front_center <- location + direction_unit_vector*(dy + h);	
			alpha <- angle;	
		}
		float t <- 2*dy;
		current <- polygon([location + {dy, -dx}, location + {dy, dx},
					  location + {-dy, dx}, location + {-dy, -dx}]) rotated_by heading; 
		right_center <- get_orthogonal_point(location, front_center, t/2, 2*dx);	
		left_center <- get_orthogonal_point(front_center, location, h + dy - t/2, 2*dx);  
		front <- polygon([front_center + {h, -dx}, front_center + {h, dx},
						  front_center + {-h, dx}, front_center + {-h, -dx}]) rotated_by alpha;					  					  		  
		left <- polygon([left_center + {(h + t/2), -1*dx}, left_center + {(h + t/2), 1*dx},
						 left_center + {-(h + t/2), 1*dx}, left_center + {-(h + t/2), -1*dx}]) rotated_by alpha;
		right <- polygon([right_center + {(h + t/2), -1*dx}, right_center + {(h + t/2), 1*dx},
						  right_center + {-(h + t/2), 1*dx}, right_center + {-(h + t/2), -1*dx}]) rotated_by alpha;
	}

//	bool check_point_overlaps_rectangle(point P, point A, point B, point C, point D) {
//		float ADP <- abs((A.x*D.y - D.x*A.y) + (D.x*P.y - P.x*D.y) + (P.x*A.y - A.x*P.y))/2;
//		float DCP <- abs((D.x*C.y - C.x*D.y) + (C.x*P.y - P.x*C.y) + (P.x*D.y - D.x*P.y))/2;
//		float CBP <- abs((C.x*B.y - B.x*C.y) + (B.x*P.y - P.x*B.y) + (P.x*C.y - C.x*P.y))/2;
//		float BAP <- abs((B.x*A.y - A.x*B.y) + (A.x*P.y - P.x*A.y) + (P.x*B.y - B.x*P.y))/2;
//		float ABCD <- distance_to(A,B)*distance_to(B,C);
//		return ADP + DCP + CBP + BAP > ABCD ? false : true;
//	}
//	
//	bool check_points_overlaps_rectangle(list<point> points, point A, point B, point C, point D) {
//		loop p over:points {
//			if check_point_overlaps_rectangle(p, A, B, C, D) { return true; }
//		}
//		return false;
//	}
	
	// ------------- CORNER ACTIONS -----------------
	action update_bezier_curve {
		point start_point <- location;
		float angle_at_corner <- angle_between(target_node, start_node, next_node);
		float dis <- distance_to(target_node, next_node) - compute_distance(location, target_node, orthogonal_target_point);
		point end_point <- get_orthogonal_point(target_node, next_node, dis, compute_distance(location, start_node, target_node));
//		float inflection <- compute_inflection(angle_at_corner, compute_distance(location, start_node, target_node), 0.35, 0.5, width_of_road_belong);
		float inflection <- compute_inflection(target_node, start_node, next_node);
		bool at_right_side <- (angle_at_corner < 180) ? true : false;  
		bezier_curve <- curve(start_point, end_point, inflection, at_right_side, 10);
	}
	
	action update_minimum_steering_distance {
		if (next_node != nil) {
			float alpha <- angle_between(target_node, start_node, next_node);
			minimum_steering_distance <- (alpha > 180 and next_status != 'at_intersection') ? width_of_road_belong/tan((360 - alpha)/2) + max_speed*STEP : max_speed*STEP;
		}
	}
	
	action assign_next_segment {
		start_node <- target_node;
		int idx <- (road_belong_nodes index_of target_node);
		if (idx < length(road_belong_nodes) - 1) {
			target_node <- road_belong_nodes[idx + 1];
		} else {
			road_belong <- get_next_road();
			do update_road_belong_nodes;
			target_node <- road_belong_nodes[1];
		}
		angle <- angle_between(start_node, start_node + {10,0}, target_node);
		direction_unit_vector <- (target_node - start_node)/distance_to(target_node, start_node);
		orthogonal_target_point <- get_orthogonal_point(start_node, target_node, 0.0, width_of_road_belong);
		status <- 'inside_road';
		next_node <- get_next_node();
		roadNode r <- (target_node = road_belong.endpoints[0].location) ? road_belong.endpoints[0] : (target_node = road_belong.endpoints[1].location ? road_belong.endpoints[1] : nil);
		next_status <- (next_node = nil) ? nil : (r = nil ? 'at_corner': (r.is_intersection ? 'at_intersection' : 'at_corner'));
		
		// remove intersection-attributes 
		intersection_belong <- nil;
		enter_conflict_space <- false;
	}
	
	action moving_at_corner {
		if (is_steering = false) {
			do update_bezier_curve;
			is_steering <- true;
		}
		do follow_bezier_curve;
		//done steering
		if location = last(bezier_curve.points) {
			is_steering <- false;
			do assign_next_segment;
			do update_minimum_steering_distance;
		}
	}
	
	action update_bezier_curve_inside_segment {
		// left-conflict vehicle or right-conflict vehicle must be nill;
		float h <- 8#m; 
		point p1 <- location;
		point p2 <- location + direction_unit_vector*h;
		point p3 <- (left_vehicle_conflict = nil and angle_between(start_node, target_node, left.location) < 180) ? left.location : right.location;
		point p4 <- p3 + direction_unit_vector*h;
		bezier_curve <- curve(p1, p2, p3, p4);
	}
	
	//------------------------------ INSIDE-ROAD ACTIONS ------------------------------
	action get_vehicles_conflict {
		float DISTANCE <- 20#m;
		list<vehicle> vehicles_surrounding <- ((vehicle at_distance DISTANCE) where (each.start_node = start_node));
		front_vehicle_conflict <- (vehicles_surrounding where (each.current overlaps self.front)) closest_to self;
		left_vehicle_conflict <- (vehicles_surrounding where (each.current overlaps self.left)) closest_to self;
		right_vehicle_conflict <- (vehicles_surrounding where (each.current overlaps self.right)) closest_to self;
	}
	
	action pass_vehicle_ahead {
		if (is_passing = false) {
			do update_bezier_curve_inside_segment;
			is_passing <- true;
		}
		do follow_bezier_curve;
		//done steering
		if (location = last(bezier_curve.points)) {
			is_passing <- false;
			bezier_curve <- nil;
		}
	}
	
	action define_target_near_intersection {
		if (next_node != nil) {
			float alpha <- angle_between(target_node, start_node, next_node);
			if (alpha < 180 and left_vehicle_conflict = nil and angle_between(start_node, target_node, left.location) < 180) {
				target <- left.location;
			} else if (alpha > 180 and right_vehicle_conflict = nil and compute_distance(right.location, start_node, target_node) < width_of_road_belong) {
				target <- right.location;
			}
		}
	}
	
	action follow_bezier_curve {
		 if (front_vehicle_conflict != nil) {
		 	do decelerate;
		 } else {
		 	do free_accelerate;
		 }
		 do follow path: path(bezier_curve) speed: speed;
	}
	
	action moving_straight {
		target <- front.location;
		if (next_status = 'at_intersection') {
			do define_target_near_intersection;
		}
		do goto target:target speed:speed;
	}
	
	// -------------- INTERSECTION ACTIONS -----------------------
	action update_bezier_curve_intersection {
		point start_point <- location;
		float angle_at_corner <- angle_between(target_node, start_node, next_node);
		float dis <- distance_to(target_node, next_node) - compute_distance(location, target_node, orthogonal_target_point);
		float d <- compute_distance(location, start_node, target_node);
		float k <- (angle_at_corner = 180) ? d : ((angle_at_corner < 180) ? d/2 : width_of_road_belong/2 + d/2) ;
		point end_point <- get_orthogonal_point(target_node, next_node, dis, k);
//		float inflection <- compute_inflection(angle_at_corner, compute_distance(location, start_node, target_node), 0.35, 0.5, width_of_road_belong);
		float inflection <- compute_inflection(target_node, start_node, next_node);
		bool at_right_side <- (angle_at_corner < 180) ? true : false;  
		bezier_curve <- curve(start_point, end_point, inflection, at_right_side, 10);
	}
	
	action follow_curve_intesection {
		if (is_steering = false) {
			do assign_strip_to_curve;
			bezier_curve <- polyline(reverse(reverse(bezier_curve.points) + location));
			is_steering <- true;
		}
		do follow path: path(bezier_curve) speed: speed;
		//done steering
		if location = last(bezier_curve.points) {
			is_steering <- false;
			do assign_next_segment;
			do update_minimum_steering_distance;
		}
	}
	
	action moving_at_intersection(traffic_light light_query, float alpha, traffic_light light, traffic_light opposite_light) {
		if (alpha <= 180) { //handle turn-left and go-straight cases
			do handle_conflict_space(light_query, alpha, light, opposite_light);
		}	
		do follow_curve_intesection;	
	}
	
	action handle_conflict_space(traffic_light light_query, float alpha, traffic_light light, traffic_light opposite_light) {
		geometry polygon_check <- current;
		if (enter_conflict_space = false) {
			do find_appropriate_speed(light, opposite_light, alpha);
			if (polygon_check overlaps intersection_belong.conflict_space_map[light_query]) {
				bool accepted;
				ask intersection_belong {
					accepted <- accept_entering_conflict_space(light_query, myself);
				}
				if (accepted = false) {
					speed <- 0.0#m/#s;
				} else {
					enter_conflict_space <- true;
					ask intersection_belong {
						conflict_list[light_query] <- conflict_list[light_query] + myself;
					}
				}
			}
		}
	}
	
	action assign_strip_to_curve {
		road current_road <- road_belong;
		road next_road <- get_next_road();
		if (next_road = nil) { write "follow_strips_intersection"; }
		int idx1 <- intersection_belong.roads index_of current_road;
		int idx2 <- intersection_belong.roads index_of next_road;
		int index_map <- intersection_belong.strips_mapping[idx2, idx1];
		list<geometry> strips <- intersection_belong.strips[index_map];
		geometry candidate <- strips closest_to self;
		bezier_curve <- candidate;
	}
	
	action follow_traffic_signal(traffic_light light_belong, traffic_light opposite_light, float alpha) {
		traffic_light light_query <- (alpha = 180) ? light_belong : opposite_light;
		if (light_belong.is_red = true and is_steering = false) {
			speed <- 0.0;
		} else {
			speed <- 10.0 #m/#s;
			do moving_at_intersection(light_query, alpha, light_belong, opposite_light);
		}
	}
	
	action find_appropriate_speed(traffic_light light, traffic_light opposite_light, float alpha) {
		if (alpha <= 180 and light.is_red = false) {
			int my_number <- alpha = 180 ? intersection_belong.conflict_info[light]['go_straight'] as int :
										   intersection_belong.conflict_info[light]['turn_left'] as int;
			int conflict_number <- alpha = 180 ? intersection_belong.conflict_info[opposite_light]['turn_left'] as int :
										   	     intersection_belong.conflict_info[opposite_light]['go_straight'] as int;
			if my_number < conflict_number {
				speed <- 2.0#m/#s;
			} 
		}
	}
	
	// ---------------- REFLEX ----------------
	reflex moving_on_road_segment when:status = 'inside_road' {	
		if (is_passing = true) {
			do pass_vehicle_ahead;
		} else {
			if (front_vehicle_conflict = nil) {
				do free_accelerate;
				do moving_straight;
			} else if ((left_vehicle_conflict = nil and angle_between(start_node, target_node, left.location) < 180 and compute_distance(left.location, start_node, target_node) > dx) or 
			(right_vehicle_conflict = nil and compute_distance(right.location, start_node, target_node) < width_of_road_belong - dx)) {
				do free_accelerate;
				do pass_vehicle_ahead;
			} else {
				do decelerate;
				do moving_straight;
			}
		}
	}
	
//	reflex stop when: status = 'stop' {
//		delay_count <- delay_count + 1;
//		if delay_count > 2/STEP {
//			delay_count <- 0;
//			status <- 'at_intersection';
//		}		
//	}
	
	reflex moving_at_corner when:status = 'at_corner'{
		do moving_at_corner;
	}
	
	reflex moving_at_intersection when:status = 'at_intersection'{
		int idx <- intersection_belong.roads index_of road_belong; 
		traffic_light light_belong <- intersection_belong.light_control[idx];
		float alpha <- (next_node != nil) ? angle_between(target_node, start_node, next_node) : 0.0;
		traffic_light opposite_light <- one_of(intersection_belong.lights_green - light_belong);
		do follow_traffic_signal(light_belong, opposite_light, alpha); 
	}

	reflex update_parameters {
		point p1 <- (next_status = 'at_intersection') ? get_orthogonal_point(start_node, target_node, width_of_road_belong, 0.0) : target_node;
		point p2 <- get_orthogonal_point(start_node, p1, 0.0, width_of_road_belong);
		float d <- compute_distance(location, p1, p2);		
		if (d < minimum_steering_distance  + 1 and is_steering = false) {
			if (next_status = 'at_corner') {
				status <- 'at_corner';
			} else if (next_status = 'at_intersection') {
				roadNode r <- (target_node = road_belong.endpoints[0].location) ? road_belong.endpoints[0] : road_belong.endpoints[1];
				intersection_belong <- r.intersection_belong;
				status  <- 'at_intersection';
			} else if (next_status = nil) {
				do die;
			}
		}
		do update_space;
		do get_vehicles_conflict;
		observing_distance <- COLLISION_AVOIDANCE_DURATION*speed;
	}
	
	aspect base {
		if debug {
			draw current color:#yellow;
			draw left color: #blue;
			draw front color:#red;
			draw right color: #green;
			draw bezier_curve color:#blue;	
		} else {
			float L <- (type = 'CAR') ? CAR_LENGTH : MOTORBIKE_LENGTH;
			float W <- (type = 'CAR') ? CAR_WIDTH : MOTORBIKE_WIDTH;
			draw image_path size:{L, W} rotate: heading;
//			draw bezier_curve color:#blue;	
		}
	}
}