/***
* Name: traffictwowayroad
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model traffictwowayroad

/* Insert your model definition here */
global {
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png")];
	
	init {
		 create road number: 1 {
		 	length <- 80 #m;
		 	width <- 6 #m;
		 	type <- "left";
		 	point p <- {0, 0};
		 	float d <- 5 #m;
		 	shape <- polygon([p, p + {length, 0}, p + {length, width}, p + {0, width}]);
		 	bound <- polygon([p + {length - d, 0}, p + {length, 0}, p + {length, width}, p + {length - d, width}]);
		 }
		 
		 create road number: 1 {
		 	length <- 80 #m;
		 	width <- 6 #m;
		 	type <- "right";
		 	point p <- {0, 6};
		 	float d <- 5 #m;
		 	shape <- polygon([p, p + {length, 0}, p + {length, width}, p + {0, width}]);
		 	bound <- polygon([p, p + {d, 0}, p + {d, width}, p + {0, width}]);
		 }
	}
	
	reflex control_traffic {
		list<vehicle> vehicle_overlap_road0 <- vehicle where (each overlaps road(0).bound);
		if length(vehicle_overlap_road0) = 0 {
			loop i from:1 to: 6 {
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
						max_speed <- rnd(0.4, 1.0) #m/#s;
					} else {
						name <- 'motorbike';
						length <- 1.8 #m;
						width <- 0.7 #m;
						df <- 0.25 #m;
						db <- 0.15 #m;
						dx <- width/2 + db;
						dy <- length/2 + df;
						speed <- 0.1;
						max_speed <- rnd(0.2, 0.7) #m/#s;
					}
					direction <- road(0).type;
					location <- any_location_in(road(0).bound);
					road_belong <- road(0);
					test <- false;
				}
			}
		}
		
		list<vehicle> vehicle_overlap_road1 <- vehicle where (each overlaps road(1).bound);
		if length(vehicle_overlap_road1) = 0 {
			loop i from:1 to: 6 {
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
						max_speed <- rnd(0.4, 1.0) #m/#s;
					} else {
						name <- 'motorbike';
						length <- 1.8 #m;
						width <- 0.7 #m;
						df <- 0.25 #m;
						db <- 0.15 #m;
						dx <- width/2 + db;
						dy <- length/2 + df;
						speed <- 0.1;
						max_speed <- rnd(0.2, 0.7) #m/#s;
					}
					direction <- road(1).type;
					location <- any_location_in(road(1).bound);
					road_belong <- road(1);
					test <- false;
				}
			}
		}
		
		ask vehicle {
			if ((self.location overlaps road(0).shape = false) and (self.location overlaps road(1).shape = false)) {
				do die;
			}
		}
	}
	
}

species road {
	string type;
	geometry bound;
	float length;
	float width;
	aspect base {
		draw shape color: #white border:#black ;
		draw bound color: #yellow;
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
	
	string direction;
	geometry current;
	geometry front;
	geometry left;
	geometry right;
	
	point target;
	bool test;
	
	road road_belong;
	
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
		return ((length(vehicle_conflict_left) = 0) and (left.location overlaps road_belong)) ? true : false;
	}
	
	bool check_turn_right {
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		return ((length(vehicle_conflict_right) = 0) and (right.location overlaps road_belong)) ? true : false;
	}
	
	action update_polygon {
		float len <- (speed/max_speed)*dy + 0.2*dy;
		
		switch direction {
			match "up" {
				current <- polygon([{location.x - dx, location.y - dy}, {location.x + dx, location.y - dy}, 
	    						  {location.x + dx, location.y + dy}, {location.x - dx, location.y + dy}]) 
	    						  rotated_by (heading + 90);
				front <- polygon([{location.x - dx, location.y - dy - len}, {location.x + dx, location.y - dy - len},
	    						  {location.x + dx, location.y - dy}, {location.x - dx, location.y - dy}]);
	    		left <- polygon([{location.x - 2*dx, location.y - dy - len}, {location.x - dx, location.y - dy - len},
	    						 {location.x - dx, location.y - dy}, {location.x - 2*dx, location.y - dy}]);
	    		right <- polygon([{location.x + 2*dx, location.y - dy - len}, {location.x + dx, location.y - dy - len},
	    						  {location.x + dx, location.y - dy}, {location.x + 2*dx, location.y - dy}]);
			}
			
			match "down" {
				current <- polygon([{location.x - dx, location.y - dy}, {location.x + dx, location.y - dy}, 
	    						  {location.x + dx, location.y + dy}, {location.x - dx, location.y + dy}]) 
	    						  rotated_by (heading + 90);
				front <- polygon([{location.x - dx, location.y + dy + len}, {location.x + dx, location.y + dy + len},
	    						  {location.x + dx, location.y + dy}, {location.x - dx, location.y + dy}]);
	    		left <- polygon([{location.x - 2*dx, location.y + dy + len}, {location.x - dx, location.y + dy + len},
	    						 {location.x - dx, location.y + dy}, {location.x - 2*dx, location.y + dy}]);
	    		right <- polygon([{location.x + 2*dx, location.y + dy + len}, {location.x + dx, location.y + dy + len},
	    						  {location.x + dx, location.y + dy}, {location.x + 2*dx, location.y + dy}]);
			}
			match "right" {
				current <- polygon([{location.x - dy, location.y - dx}, {location.x + dy, location.y - dx},
									{location.x + dy, location.y + dx}, {location.x - dy, location.y + dx}]);
				
				front <- polygon([{location.x + dy, location.y - dx}, {location.x + dy + len, location.y - dx},
								  {location.x + dy + len, location.y + dx}, {location.x + dy, location.y + dx}]);
				left <- polygon([{location.x + dy, location.y - 2*dx}, {location.x + dy + len, location.y - 2*dx},
								 {location.x + dy + len, location.y - dx}, {location.x + dy, location.y - dx}]);
				right <- polygon([{location.x + dy, location.y + 2*dx}, {location.x + dy + len, location.y + 2*dx},
								 {location.x + dy + len, location.y + dx}, {location.x + dy, location.y + dx}]);
			}
			match "left" {
				current <- polygon([{location.x - dy, location.y - dx}, {location.x + dy, location.y - dx},
									{location.x + dy, location.y + dx}, {location.x - dy, location.y + dx}]);
				
				front <- polygon([{location.x - dy, location.y - dx}, {location.x - dy - len, location.y - dx},
								  {location.x - dy - len, location.y + dx}, {location.x - dy, location.y + dx}]);
				left <- polygon([{location.x - dy, location.y - 2*dx}, {location.x - dy - len, location.y - 2*dx},
								 {location.x - dy - len, location.y - dx}, {location.x - dy, location.y - dx}]);
				right <- polygon([{location.x - dy, location.y + 2*dx}, {location.x - dy - len, location.y + 2*dx},
								 {location.x - dy - len, location.y + dx}, {location.x - dy, location.y + dx}]);
			}
		}
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

experiment my_experiment {
	float minimum_cycle_duration <- 0.1;
	output {
		display my_display {
			species road aspect: base;
			species vehicle aspect: base;
		}
	}
}