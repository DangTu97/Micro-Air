/**
* Name: road
* Based on the internal empty template. 
* Author: tu
* Tags: 
*/


model road
import 'vehicle.gaml'
/* Insert your model definition here */

species road {
	bool is_twoway;
	float width;
	geometry geom_display;
	rgb color <- #white;
	traffic_light light_control;
	string junction;
	list<roadNode> endpoints; // two endpoints;
	
	aspect base {
		draw geom_display color:color border:#white;
		if is_twoway { draw shape color: #grey; }
//		if (junction = 'roundabout') { draw geom_display color:#green; }
	}
}

species traffic_light {
	int counter;
	rgb color;
	int t_g;
	int t_y;
	int t_r;
	
	// additional variables
	bool is_green;
	bool is_yellow;
	bool is_red;
	road road_control;
	
	reflex change_color {
		counter <- counter + 1;
        if (counter >= t_g + t_y + t_r) { 
            counter <- 0;
        } else if (counter <= t_g ) {
        	color <- #green;
        	is_green <- true;
        	is_yellow <- false;
        	is_red <- false;
        } else if ((counter > t_g) and (counter <= t_g + t_y)) {
        	color <- #yellow;
        	is_green <- false;
        	is_yellow <- true;
        	is_red <- false;
        } else {
        	color <- #red;
        	is_green <- false;
        	is_yellow <- false;
        	is_red <- true;
        }
	}
	
	aspect base {
		draw shape at:location color:color;
	}
}

species roadNode {
	bool is_intersection;
	geometry geom_display;
	intersection intersection_belong;
	aspect base {
		draw geom_display color:is_intersection ? #red : #green;
	}
}

species intersection {
	point center_location;
	list<road> roads;
	list<point> attached_points;
	list<traffic_light> light_control;
	float distance_from_center;
	list<int> n_strips; //number of strips for left, opposite, right;
	list<float> percents;
	matrix<int> strips_mapping;
	list<list<geometry>> strips;
	map<traffic_light, traffic_light> light_pairs;
	list<traffic_light> lights_green;
	
	//testing 
	map<traffic_light, map> light_vehicle_map; //list of [straight, left, right]-vehicles
	map<traffic_light, map> conflict_info;
	map<traffic_light, geometry> conflict_space_map;
	map<traffic_light, list<vehicle>> conflict_list;
	map<traffic_light, road> conflict_space_occupied;
	
	point get_orthogonal_point(point s, point t, float d1, float d2) {
		// d1: distance from target
		// d2: distance to segment (s, t)
		float a <- (t - s).x;
		float b <- (t - s).y;    	
		point p <- t - (t - s)*d1/distance_to(s, t) + {-b*d2/sqrt(a*a + b*b), a*d2/sqrt(a*a + b*b)};
		return p;
	}
	
	action build_attached_points {
		loop r over:roads {
			list<point> nodes <- (center_location = first(r.shape.points)) ? r.shape.points : reverse(r.shape.points);
			point neighbor_point <- center_location + (nodes[1] - center_location)*distance_from_center/distance_to(nodes[1], center_location);
			attached_points <- attached_points + neighbor_point;
		}
	}

	action build_geometry_matrix {
		strips_mapping <- 0 as_matrix({length(roads), length(roads)});
		loop idx1 from:0 to:length(roads) - 1 {
			road r1 <- roads[idx1];
			point orthogonal_r1 <- get_orthogonal_point(attached_points[idx1], center_location, distance_from_center, r1.width);
			int count <- 0;
			strips <- strips + [nil];
			strips_mapping[idx1, idx1] <- length(strips) - 1;
			float percent_sum <- 0.0;
			loop j from:0 to:length(roads) - 2 {
				int idx2 <- (idx1 + j + 1) mod length(roads);
				road r2 <- roads[idx2];
				list<geometry> t;
				point orthogonal_r2 <- get_orthogonal_point(center_location, attached_points[idx2], 0.0, r2.width);
				loop k from:1 to:n_strips[count] {
					point p1 <- attached_points[idx1] + (orthogonal_r1 - attached_points[idx1])*(percent_sum + k/(n_strips[count])*percents[count]);
					point p2 <- attached_points[idx2] + (orthogonal_r2 - attached_points[idx2])*(percent_sum + k/(n_strips[count])*percents[count]);
					float alpha <- angle_between(center_location, attached_points[idx1], attached_points[idx2]);
					float inflection <- (alpha < 205.0 and alpha > 155.0) ? 0.0 : 0.3;
					bool at_right_side <- (alpha <= 180) ? true : false;
					t <- t + curve(p1, p2, inflection, at_right_side);
				}
				percent_sum <- percent_sum + percents[count];	
				count <- count + 1;
				strips <- strips + [t];
				strips_mapping[idx2, idx1] <- length(strips) - 1;
			}
		}
	}
	
	action get_conflict_space_map {
		list<geometry> conflict_space;
		loop r over:roads {
			int idx <- roads index_of r;
//			point p1 <- get_orthogonal_point(attached_points[idx], center_location, distance_from_center*percents[2], percents[0]*r.width);
//			point p2 <- get_orthogonal_point(attached_points[idx], center_location, distance_from_center*percents[2], (percents[0] + percents[1])*r.width);
			point p1 <- get_orthogonal_point(attached_points[idx], center_location, distance_from_center*0.4, percents[0]*r.width);
			point p2 <- get_orthogonal_point(attached_points[idx], center_location, distance_from_center*0.4, (percents[0] + percents[1])*r.width);
			point p3 <- p2 + (center_location - attached_points[idx])*1;
			point p4 <- p1 + (center_location - attached_points[idx])*1;
			conflict_space <- conflict_space + polygon([p1, p2, p3, p4, p1]);
		}
		conflict_space_map <- create_map(light_control, conflict_space);
	}
	
	action get_vehicles_at_intersection {
		float d_check <- 30#m;
		list<vehicle> vehicles_at_intersection <- (vehicle at_distance d_check);
		lights_green <- light_control where (each.is_red = false);
		list<list<vehicle>> vehicles_conflict;
		light_vehicle_map <- [];		
		loop l over:lights_green {
			map<string, list<vehicle>> test;
			map<string, int> counts;
			
			list<vehicle> vehicles_belong_l <- vehicles_at_intersection where (each.road_belong = l.road_control and each.target_node = center_location);
			test['turn_left'] <- vehicles_belong_l where (angle_between(each.target_node, each.start_node, each.next_node) < 180);
			test['go_straight'] <- vehicles_belong_l where (angle_between(each.target_node, each.start_node, each.next_node) = 180);
			test['turn_right'] <- vehicles_belong_l where (angle_between(each.target_node, each.start_node, each.next_node) > 180);
			light_vehicle_map[l] <- test;
			
			counts['turn_left'] <- length(test['turn_left']);
			counts['go_straight'] <- length(test['go_straight']);
			counts['turn_right'] <- length(test['turn_right']);
			conflict_info[l] <- counts;
		}
	}
	
	action build_conflict_list {
		conflict_list <- create_map(light_control, list_with(length(light_control), nil));
	}
	
	action control_conflict_list {
		loop l over: light_control {
			loop ve over:conflict_list[l] {
				if (dead(ve) or (ve.current overlaps conflict_space_map[l]) = false) {
					conflict_list[l] <- conflict_list[l] - ve;
				}
			}
		}
	}
	
	action control_flag {
		list<road> my_roads;
		loop l over:lights_green {
			conflict_space_occupied[l] <-  length(conflict_list[l]) = 0 ? nil : last(conflict_list[l]).road_belong;
		}
	}
	
	bool continue_moving(traffic_light light_query, vehicle ve) {
		if length(conflict_list[light_query]) = 0 {
			return true;
		} else {
			loop v over:conflict_list[light_query] {
				if v.start_node != ve.start_node {
					return false;
				}
			}
		}
		return true;
	}
	
	bool accept_entering_conflict_space(traffic_light light_query, vehicle ve) {
		return conflict_space_occupied[light_query] = nil or conflict_space_occupied[light_query] = ve.road_belong ? true : false;
	}
	
	reflex control_traffic {
		do get_vehicles_at_intersection;
		do get_conflict_space_map;
		do control_conflict_list;
		do control_flag;
	}
	
	
	aspect base {
		draw circle(0.5#m) at:location color: #green;
//		loop p over:attached_points {
//			draw circle(0.5#m) at:p color:#red;
//		}
		loop strip over:strips {
			loop s over:strip {
				draw s color:#red;
			}
		}
		loop l over:lights_green {
			draw conflict_space_map[l] color:#blue;
		}
	}	
}












