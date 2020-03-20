/***
* Name: testnewmodel3
* Author: dang tu
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model testnewmodel3

/* Insert your model definition here */

global {
	list<image_file> images <- [image_file("../includes/car.png"), image_file("../includes/motorbike.png")];
	
	init {
		ask cell {
			if grid_x = 10 {
				is_road <- true;
			}
		}
		
		create vehicle number: 1 {
			name <- 'car';
			length <- 3.8 #m;
			width <- 1.5 #m;
			df <- 0.5 #m;
			db <- 0.2 #m;
			location <- cell[10, 12].location - {1.0, 0};
			dx <- width/2 + db;
			dy <- length/2 + df;
			speed <- 0.25;
			test <- true;
		}
		
		create vehicle number: 1 {
			name <- 'motorbike';
			length <- 1.8 #m;
			width <- 0.7 #m;
			df <- 0.25 #m;
			db <- 0.15 #m;
			location <- cell[10, 10].location + {1.68, 0};
			dx <- width/2 + db;
			dy <- length/2 + df;
			speed <- 0.25;
			test <- true;
		}
			
		create vehicle number: 1 {
			name <- 'car';
			length <- 3.8 #m;
			width <- 1.5 #m;
			df <- 0.5 #m;
			db <- 0.2 #m;
			location <- cell[10, 15].location - {0.8, 0};
			dx <- width/2 + db;
			dy <- length/2 + df;
			speed <- 0.1;
			test <- false;
		}
		
		create vehicle number: 1 {
			name <- 'car';
			length <- 3.8 #m;
			width <- 1.5 #m;
			df <- 0.5 #m;
			db <- 0.2 #m;
			location <- cell[10, 16].location;
			dx <- width/2 + db;
			dy <- length/2 + df;
			speed <- 0.1;
			test <- false;
		}
		
		loop i from:17 to:24 {
			create vehicle number: 1 {
				name <- 'motorbike';
				length <- 1.8 #m;
				width <- 0.7 #m;
				df <- 0.25 #m;
				db <- 0.15 #m;
				if mod(i,2) = 0 {
					location <- cell[10, i].location - {rnd(1.5), rnd(1.0)};
				} else {
					location <- cell[10, i].location + {rnd(1.5), rnd(1.0)};
				}
				dx <- width/2 + db;
				dy <- length/2 + df;
				speed <- 0.1;
				test <- false;
			}
		}
	}
}

grid cell cell_width: 6 cell_height: 4 {
	bool is_road;
	reflex update_color{
		color <- is_road ? #white : #grey;
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
		cell left_on_cell <- first(cell where (each overlaps left.location));
		return ((length(vehicle_conflict_left) = 0) and (left_on_cell.is_road = true)) ? true : false;
	}
	
	bool check_turn_right {
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		cell right_on_cell <- first(cell where (each overlaps right.location));
		return ((length(vehicle_conflict_right) = 0) and (right_on_cell.is_road = true)) ? true : false;
	}
	
	action update_polygon {
	    geometry var0 <- polygon([{location.x - dx, location.y - dy}, {location.x + dx, location.y - dy}, 
	    {location.x + dx, location.y + dy}, {location.x - dx, location.y + dy}]) rotated_by (heading + 90);
	    current <- var0;
	    
	    front <- polygon([{location.x - dx, location.y - 3*dy}, {location.x + dx, location.y - 3*dy},
	    	{location.x + dx, location.y - dy}, {location.x - dx, location.y - dy}]);
	    
	    //origin
//	    left <- polygon([{location.x - 3*dx, location.y - 3*dy}, {location.x - dx, location.y - 3*dy},
//	    	{location.x - dx, location.y - dy}, {location.x - 3*dx, location.y - dy}]);
//	    	
//	    right <- polygon([{location.x + dx, location.y - 3*dy}, {location.x + 3*dx, location.y - 3*dy},
//	    	{location.x + 3*dx, location.y - dy}, {location.x + dx, location.y - dy}]);
		
		//improve
	    left <- polygon([{location.x - 2*dx, location.y - 3*dy}, {location.x - dx, location.y - 3*dy},
	    	{location.x - dx, location.y - dy}, {location.x - 2*dx, location.y - dy}]);
	    	
	    right <- polygon([{location.x + dx, location.y - 3*dy}, {location.x + 2*dx, location.y - 3*dy},
	    	{location.x + 2*dx, location.y - dy}, {location.x + dx, location.y - dy}]);
	    	
	    target <- front.location;
	}
	
	action speed_up {
		speed <- min(1.0, speed + 0.05);
	}
	
	action slow_down(vehicle vehicle_ahead) {
		speed <- max(vehicle_ahead.speed/2, speed - 0.5);
		//speed <- max(vehicle_ahead.speed/2, 0.05);
	}
	
	reflex move when:target != nil {
		list<vehicle> vehicle_conflict_front <- get_vehicle_conflict_front();
		list<vehicle> vehicle_conflict_left <- get_vehicle_conflict_left();
		list<vehicle> vehicle_conflict_right <- get_vehicle_conflict_right();
		write "------";
		write self;
		write vehicle_conflict_front;
		write vehicle_conflict_left;
		write vehicle_conflict_right;

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
	
	aspect visual {
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
	float minimum_cycle_duration <- 0.25;
	output {
		display my_display {
			grid cell lines:#white;
			species vehicle aspect: visual;
			
		}
	}
}