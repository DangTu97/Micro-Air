/***
* Name: trafficdynamic
* Author: dang tu
* Description: moving on rectangle road with dynamic polygon
* Tags: Tag1, Tag2, TagN
***/

model trafficdynamic

/* Insert your model definition here */

global {
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png")];
	float environment_size <- 200.0;
	geometry shape <- square(environment_size);
	init {
		create road number: 1 {
			width <- 6 #m;
			length <- 150 #m;
			float l <- road(0).length;
			float w <- road(0).width;
			point p <- road(0).location;
			shape <- polygon([p - {w/2,l/2}, p + {w/2,-l/2}, p + {w/2,l/2}, p + {-w/2,l/2}]);
			float d <- 5 #m;
			check_boundary <- polygon([p + {-w/2,l/2 - d}, p + {w/2,l/2 - d}, p + {w/2,l/2}, p + {-w/2,l/2}]);
		}
	}
	
	reflex control_traffic {
		list<vehicle> vehicle_overlap <- vehicle where (each overlaps road(0).check_boundary);
		if length(vehicle_overlap) = 0 {
			loop i from:1 to: 5 {
				create vehicle number:1 {
				name <- flip(0.2) ? 'car' : 'motorbike';
				if name = 'car' {
					length <- 3.8 #m;
					width <- 1.5 #m;
					df <- 0.5 #m;
					db <- 0.2 #m;
					dx <- width/2 + db;
					dy <- length/2 + df;
					speed <- 0.1;
//					max_speed <- 1.0 #m/#s;
					max_speed <- rnd(0.5, 1.0) #m/#s;
				} else {
					name <- 'motorbike';
					length <- 1.8 #m;
					width <- 0.7 #m;
					df <- 0.25 #m;
					db <- 0.15 #m;
					dx <- width/2 + db;
					dy <- length/2 + df;
					speed <- 0.1;
//					max_speed <- 0.8 #m/#s;
					max_speed <- rnd(0.2, 0.6) #m/#s;
				}
				
				location <- any_location_in(road(0).check_boundary);
				heading <- 270.0;
//				test <- flip(0.9) ? false : true;
				test <- false;
			}
			}
		}
		
		ask vehicle {
			if (self.location overlaps road(0).shape = false) {
				do die;
			}
		}
	}
}


species vehicle skills:[moving] {
	string name;
	float length;
	float width;
	float max_speed;
	float df;
	float db;
	float dx;
	float dy;
	
	geometry current;
	geometry front;
	geometry left;
	geometry right;
	
	point target;
	bool test;
	
	list<vehicle> get_vehicle_conflict_front {
		list<vehicle> vehicle_conflict_front <- (vehicle at_distance(10 + dy)) where (each.current overlaps self.front);
		return vehicle_conflict_front;
	}
	
	list<vehicle> get_vehicle_conflict_left {
		list<vehicle> vehicle_conflict_left <- (vehicle at_distance(10 + dy)) where (each.current overlaps self.left);
		return vehicle_conflict_left;
	}
	
	list<vehicle> get_vehicle_conflict_right {
		list<vehicle> vehicle_conflict_right <- (vehicle at_distance(10 + dy)) where (each.current overlaps self.right);
		return vehicle_conflict_right;
	}
	
	bool check_go_straight {
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
		return (length(vehicle_conflict_front) = 0) ? true : false;
	}
	
	bool check_turn_left {
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		return ((length(vehicle_conflict_left) = 0) and (left.location overlaps road(0))) ? true : false;
	}
	
	bool check_turn_right {
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		return ((length(vehicle_conflict_right) = 0) and (right.location overlaps road(0))) ? true : false;
	}
	
	action update_polygon {
	    geometry var0 <- polygon([{location.x - dx, location.y - dy}, {location.x + dx, location.y - dy}, 
	    {location.x + dx, location.y + dy}, {location.x - dx, location.y + dy}]) rotated_by (heading + 90);
	    current <- var0;
	    
	    float len <- (speed/max_speed)*dy + 0.2*dy;
	    
	    front <- polygon([{location.x - dx, location.y - dy - len}, {location.x + dx, location.y - dy - len},
	    	{location.x + dx, location.y - dy}, {location.x - dx, location.y - dy}]);
	    
	    //origin
//	    left <- polygon([{location.x - 3*dx, location.y - 3*dy}, {location.x - dx, location.y - 3*dy},
//	    	{location.x - dx, location.y - dy}, {location.x - 3*dx, location.y - dy}]);
//	    	
//	    right <- polygon([{location.x + dx, location.y - 3*dy}, {location.x + 3*dx, location.y - 3*dy},
//	    	{location.x + 3*dx, location.y - dy}, {location.x + dx, location.y - dy}]);
		
		//improve
	    left <- polygon([{location.x - 2*dx, location.y - dy - len}, {location.x - dx, location.y - dy - len},
	    	{location.x - dx, location.y - dy}, {location.x - 2*dx, location.y - dy}]);
	    	
	    right <- polygon([{location.x + 2*dx, location.y - dy - len}, {location.x + dx, location.y - dy - len},
	    	{location.x + dx, location.y - dy}, {location.x + 2*dx, location.y - dy}]);
	    	
	    target <- front.location;
	}
	
	action speed_up {
		speed <- min(1.0, speed + 0.05);
	}
	
	action slow_down(vehicle vehicle_ahead) {
		speed <- max(vehicle_ahead.speed/2, 0);
		//speed <- max(vehicle_ahead.speed/2, 0.05);
	}
	
	reflex move when:target != nil {
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
//		write "------";
//		write self;
//		write name;
//		write max_speed;
//		write vehicle_conflict_front;
//		write vehicle_conflict_left;
//		write vehicle_conflict_right;
		
		if (speed > max_speed) {
			speed <- max_speed;
		}

		if test = false {
			if (check_go_straight() = true) {
				do speed_up;
			} else if (check_turn_left() = true) {
				do speed_up;
				target <- left.location;
			} else if (check_turn_right() = true) {
				do speed_up;
				target <- right.location;
			} else {
//				target <- front.location + {rnd(0.5), 0};
				target <- front.location ;
				do slow_down(first(get_vehicle_conflict_front()));
			}
			
			do goto target: target speed:speed;
		}
		
		if test = true {
			target <- front.location;	
			do goto target: target speed:speed;
		}
	}
	
	reflex update_line {
		do update_polygon;
	}
	
	aspect base {
//		draw current color:#red;
//		draw front color: #blue;
//		draw left color: #grey;
//		draw right color: #yellow;
		if (name = 'car') {
			draw images[0] size: {length, width} rotate:heading;
		}
		
		if (name = 'motorbike') {
			draw images[1] size: {length, width} rotate:heading;
		}
	}
}

species road {
	init {
		location <- {100, 100};
	}
	geometry check_boundary;
	float length;
	float width;
	aspect base {
		draw shape color: #white border:#grey;
		draw check_boundary color:#yellow;
	}
}

experiment my_experiment {
	float minimum_cycle_duration <- 0.05;
	output {
		display my_display {
			species road aspect: base;
			species vehicle aspect: base;
		}
	}
}